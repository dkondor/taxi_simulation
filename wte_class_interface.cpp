/*
 * wte_class_interface.cpp
 * 
 * Copyright 2021 Daniel Kondor <kondor.dani@gmail.com>
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following disclaimer
 *   in the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of the  nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 */


#include "trip.h"
#include "wte_class.h"
#include "wte_class_interface.h"
#include "iterator_zip.h"
#include "read_table.h"
#include <stdlib.h>
#include <string.h>
#include <unordered_set>

/* "wrapper" derived class, with extra functionality to hold the result of the last exception (if any) */
template<class base>
class wte_trips2 : public base {
	public:
		constexpr static size_t buffer_size = 1024;
		char static_buffer[buffer_size];
		char* last_exception;
		char* dynamic_buffer;
		size_t dynamic_buffer_len;
		bool handle_exceptions = true;
		
		wte_trips2():last_exception(0),dynamic_buffer(0) { }
		template<class... Args>
		explicit wte_trips2(Args&&... args):base(std::forward<Args>(args)...),last_exception(0),dynamic_buffer(0) { }
		~wte_trips2() { if(dynamic_buffer) free(dynamic_buffer); dynamic_buffer = 0; }
		
		/* handle previous exception by copying the error message to one of the buffers */
		void handle_exception(const char* what) {
			if(!what) { last_exception = 0; return; }
			size_t len = strlen(what);
			if(len+1 < buffer_size) {
				strcpy(static_buffer,what);
				last_exception = static_buffer;
				return;
			}
			if(len+1 < dynamic_buffer_len) {
				strcpy(dynamic_buffer,what);
				last_exception = dynamic_buffer;
				return;
			}
			char* tmp = (char*)realloc(dynamic_buffer,len+1);
			if(tmp) {
				dynamic_buffer = tmp;
				strcpy(dynamic_buffer,what);
				last_exception = dynamic_buffer;
				return;
			}
			/* here, we cannot allocate memory, use just the static buffer */
			strncpy(static_buffer,what,buffer_size-1);
			static_buffer[buffer_size-1] = 0;
			static_buffer[buffer_size-2] = '.';
			static_buffer[buffer_size-3] = '.';
			static_buffer[buffer_size-4] = '.';
			last_exception = static_buffer;
		}
};


uint64_t trip_get_id(const void* t) { return ((const trip*)t)->id; }
unsigned int trip_get_start_ts(const void* t) { return ((const trip*)t)->start_ts; }
unsigned int trip_get_end_ts(const void* t) { return ((const trip*)t)->end_ts; }
unsigned int trip_get_start_node(const void* t) { return ((const trip*)t)->start_node; }
unsigned int trip_get_end_node(const void* t) { return ((const trip*)t)->end_node; }

void* trip_new(uint64_t id, unsigned int start_ts, unsigned int end_ts, unsigned int start_node, unsigned int end_node) {
	trip* t = 0;
	try { t = new trip; }
	catch(...) { return 0; }
	if(!t) return 0;
	t->id = id;
	t->start_ts = start_ts;
	t->end_ts = end_ts;
	t->start_node = start_node;
	t->end_node = end_node;
	return (void*)t;
}
void trip_free(void* t) {
	if(!t) return;
	trip* t2 = (trip*)t;
	delete t2;
}



void* wte_new() {
	wte_trips2<wte_trips>* w = 0;
	try { w = new wte_trips2<wte_trips>(); }
	catch(...) { return 0; }
	return (void*)w;
}
void wte_free(void* p) {
	if(p) {
		wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
		delete w;
	}
}

void wte_set_handle_exceptions(void* p, int should_handle_exceptions) {
	if(!p) return;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	w->handle_exceptions = should_handle_exceptions;
}

int wte_get_handle_exceptions(const void* p) {
	const wte_trips2<wte_trips>* w = (const wte_trips2<wte_trips>*)p;
	return w->handle_exceptions;
}

/* The following macros call a member function of w (which should be a wte_trips2 class)
 * and optionally catches any exceptions if w->handle_exceptions == true.
 * f should be the name of a member function with arguments (in parentheses).
 * The second version stores the return value in the supplied variable r. */
#define TRY_CALL(w, f) if(w->handle_exceptions) try { w->f; } catch(const std::exception& e) { w->handle_exception(e.what()); return 0; } else w->f;
#define TRY_CALL_RET(w, f, r) if(w->handle_exceptions) try { r = w->f; } catch(const std::exception& e) { w->handle_exception(e.what()); return 0; } else r = w->f;


int wte_load_index(void* p, unsigned int matrix_size, const char* fnbase, const char* fnext, const char* fnbase_sorted,
		const char* fnext_sorted, const char* fnbase_paths, const char* fnext_paths, int preread) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	TRY_CALL(w, load_index(matrix_size,fnbase,fnext,fnbase_sorted,fnext_sorted,fnbase_paths,fnext_paths,preread))
	return 1;
}

int wte_load_distances(void* p, const char* fn, int preread) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	TRY_CALL(w, load_distances(fn, preread))
	return 1;
}

int wte_process_trip(void* p, const void* t, unsigned int tmax1, int allow_moving, int* res1) {
	if(!(p && t)) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	bool res = false;
	TRY_CALL_RET(w, process_trip((const trip*)t, tmax1, allow_moving), res)
	*res1 = res;
	return 1;
}

int wte_reset(void* p, int clear_drivers) {
	if(!p) return 0;
	bool cd = false;
	if(clear_drivers) cd = true;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	TRY_CALL(w, reset(cd))
	return 1;
}

int wte_add_driver(void* p, unsigned int node, unsigned int start_time) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	TRY_CALL(w, add_driver(node,start_time))
	return 1;
}

const void* wte_get_last_result(const void* p) {
	const wte_trips2<wte_trips>* w = (const wte_trips2<wte_trips>*)p;
	return &(w->get_last_res());
}


void wte_set_seed(void* p, uint64_t seed) {
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	w->seed(seed);
}
	
int wte_read_trips(void* p, const char* fn) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	TRY_CALL(w, read_trips(fn))
	return 1;
}

int wte_read_trips_csv(void* p, const char* fn, const char* compress, unsigned int tmin, unsigned int tmax,
		int tlimit_in_day, unsigned int length_min, unsigned int length_max, unsigned int dist_min,
		unsigned int dist_max, double speed_min, double speed_max, int clear_trips,
		const char* node_filter_fn, int chicago_format) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	
	wte_trips::read_csv_filter filt;
	filt.tmin = tmin;
	filt.tmax = tmax;
	filt.tlimit_in_day = tlimit_in_day;
	filt.length_min = length_min;
	filt.length_max = length_max;
	filt.dist_min = dist_min;
	filt.dist_max = dist_max;
	filt.speed_min = speed_min;
	filt.speed_max = speed_max;
	filt.nodes_file = node_filter_fn;
	
	TRY_CALL(w, read_trips_csv(fn, filt, compress, clear_trips, chicago_format))
	return 1;
}

int wte_add_trip(void* p, const void* t) {
	if(!(p && t)) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	const trip* t2 = (const trip*)t;
	TRY_CALL(w, add_trip(t2))
	return 1;
}

void wte_sort_trips(void* p, int reassign_ids) {
	if(!p) return;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	w->sort_trips(reassign_ids);
}

int wte_is_sorted(const void* p) {
	if(!p) return 0;
	const wte_trips2<wte_trips>* w = (const wte_trips2<wte_trips>*)p;
	return w->is_sorted();
}
	
	
int wte_clear(void* p) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	TRY_CALL(w, clear())
	return 1;
}


int wte_add_drivers(void* p, uint64_t n, double tavg, int use_end_locations) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	bool p1 = (use_end_locations != 0);
	TRY_CALL(w, add_drivers(n,tavg,p1))
	return 1;
}
		
int wte_process_next_trip(void* p, unsigned int tmax1, int* res1) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	bool res = false;
	TRY_CALL_RET(w, process_next_trip(tmax1), res)
	*res1 = res;
	return 1;
}
int wte_is_end(const void* p) {
	const wte_trips2<wte_trips>* w = (const wte_trips2<wte_trips>*)p;
	return w->is_end();
}
	

const void* wte_get_trip(const void* p, uint64_t idx) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	const trip* t = 0;
	TRY_CALL_RET(w, get_trip(idx), t)
	return (const void*)t;
}

uint64_t wte_ntrips(const void* p) {
	if(!p) return 0;
	const wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	return w->ntrips();
}
	

char* wte_get_error(void* p) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	return w->last_exception;
}

uint64_t wte_get_trip_idx(const void* p) {
	if(!p) return 0;
	const wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	return w->trip_idx();
}


unsigned int wte_get_total_drivers(const void* p) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	return w->n_total_drivers();
}

unsigned int wte_get_total_driver_ids(const void* p) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	return w->n_total_drivers_added();
}

const void* wte_get_driver_info(void* p, unsigned int d) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	if(w->handle_exceptions) {
		try {
			const auto& res = w->get_driver_summary(d);
			return &res;
		}
		catch(const std::exception& e) {
			w->handle_exception(e.what());
			return 0;
		}
	}
	else {
		const auto& res = w->get_driver_summary(d);
		return &res;
	}
}

/* cruising strategy */
enum wte_cruising_strategy wte_get_cruising_strategy(const void* p) {
	if(!p) return WTE_CRUISING_STRATEGY_NONE;
	const wte_trips2<wte_trips>* w = (const wte_trips2<wte_trips>*)p;
	return (enum wte_cruising_strategy)w->driver_strategy();
}
void wte_set_cruising_strategy(void* p, enum wte_cruising_strategy strategy) {
	if(!p) return;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	w->driver_strategy() = (wte::cruising_strategy)strategy;
}
	
int wte_set_cruising_target_dist(void* p, unsigned int hour, unsigned int* nodes, double* weights, size_t len) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	if(hour > 23) {
		w->handle_exception("wte_set_cruising_target_dist(): invalid hour parameter!\n");
		return 0;
	}
	TRY_CALL(w, set_cruising_target_dist((uint8_t)hour, zi::make_zip_it(nodes, weights),
			zi::make_zip_it(nodes + len, weights + len)))
	return 1;
}

int wte_read_cruising_target_dist(void* p, unsigned int hour, const char* fn) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	if(hour > 23) {
		w->handle_exception("wte_read_cruising_target_dist(): invalid hour parameter!\n");
		return 0;
	}
	TRY_CALL(w, read_cruising_target_dist((uint8_t)hour, fn))
	return 1;
}

int wte_read_cruising_target_dist_mult(void* p, const char* fn) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	TRY_CALL(w, read_cruising_target_dist_mult(fn))
	return 1;
}


/* filter the trips in the current instance */
void wte_filter_trips(void* p, const uint64_t* ids, size_t len) {
	if(!(p && ids && len)) return;
	std::unordered_set<uint64_t> ids2(ids, ids + len);
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	w->filter_trips(ids2);
}
/* make a copy with filtering the trips */
void* wte_copy_trips_filter(void* p, const uint64_t* ids, size_t len) {
	if(!p) return nullptr;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	wte_trips2<wte_trips>* w2 = nullptr;
	if(w->handle_exceptions) {
		try {
			if(ids && len) {
				std::unordered_set<uint64_t> ids2(ids, ids + len);
				w2 = new wte_trips2<wte_trips>(*w, ids2);
			}
			else w2 = new wte_trips2<wte_trips>(*w);
		}
		catch(const std::exception& e) { w->handle_exception(e.what()); return nullptr; }
	}
	else {
		if(ids && len) {
			std::unordered_set<uint64_t> ids2(ids, ids + len);
			w2 = new wte_trips2<wte_trips>(*w, ids2);
		}
		else w2 = new wte_trips2<wte_trips>(*w);
	}
	return (void*)w2;
}



/* get the number of idle drivers */
unsigned int wte_n_idle_drivers(const void* p) {
	if(!p) return 0;
	const wte_trips2<wte_trips>* w = (const wte_trips2<wte_trips>*)p;
	return w->n_idle_drivers();
}
/* get the number of repositioning drivers */
unsigned int wte_n_moving_drivers(const void* p) {
	if(!p) return 0;
	const wte_trips2<wte_trips>* w = (const wte_trips2<wte_trips>*)p;
	return w->n_moving_drivers();
}

/* start iteration on idle drivers */
void wte_idle_drivers_start(void* p) {
	if(!p) return;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	w->idle_drivers_start();
}

/* return if the end has already be reached when iterating on idle drivers (also returns 1 if the iterator is invalid) */
int wte_idle_drivers_is_end(const void* p) {
	if(!p) return 1;
	const wte_trips2<wte_trips>* w = (const wte_trips2<wte_trips>*)p;
	return w->idle_drivers_is_end();
}

/* Advances the iterator for idle drivers; returns 1 if a valid element is available or 0
 * if the end has been already reached or if the iterator is invalid. */
int wte_idle_drivers_advance(void* p) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	bool ret = false;
	TRY_CALL_RET(w, idle_drivers_advance(), ret);
	return ret;
}

/* Stores the ID and node ID of the current idle driver (pointed to be the internal iterator) in the given locations.
 * Returns 0 if the iterator is invalid or the end has already been reached; otherwise returns 1. */
int wte_idle_drivers_current(void* p, unsigned int* driver_id1, unsigned int* node_id) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	std::pair<wte::driver_id, wte::index_t> res;
	TRY_CALL_RET(w, idle_drivers_current(), res)
	*driver_id1 = res.first;
	*node_id = res.second;
	return 1;
}

/* start iteration on currently moving drivers */
void wte_moving_drivers_start(void* p) {
	if(!p) return;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	w->moving_drivers_start();
}

/* return if the end has already be reached when iterating on moving drivers (also returns 1 if the iterator is invalid) */
int wte_moving_drivers_is_end(const void* p) {
	if(!p) return 1;
	const wte_trips2<wte_trips>* w = (const wte_trips2<wte_trips>*)p;
	return w->moving_drivers_is_end();
}

/* Advances the iterator for moving drivers; returns 1 if a valid element is available or 0
 * if the end has been already reached or if the iterator is invalid. */
bool wte_moving_drivers_advance(void* p) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	bool ret = false;
	TRY_CALL_RET(w, moving_drivers_advance(), ret)
	return ret;
}

/* Stores the ID and node ID of the current moving driver (pointed to be the internal iterator) in the given locations.
 * Returns 0 if the iterator is invalid or the end has already been reached; otherwise returns 1. */
int wte_moving_drivers_current(void* p, unsigned int* driver_id1, unsigned int* node_id) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	std::pair<wte::driver_id, wte::index_t> res;
	TRY_CALL_RET(w, moving_drivers_current(), res)
	*driver_id1 = res.first;
	*node_id = res.second;
	return 1;	
}


/* start moving a driver toward a destination using the shortest path;
 * removes the given driver from either the list of idle or currently
 * moving drivers; returns 0 if driver is not an idle or
 * currently moving driver */
int wte_reposition_driver(void* p, unsigned int id, unsigned int target) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	TRY_CALL(w, reposition_driver(id, target))
	return 1;
}

/* remove a driver with the given ID; note: only possible if the driver is not assigned to serve a trip,
 * i.e. they are idle or cruising */
int wte_remove_driver(void* p, unsigned int id) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	TRY_CALL(w, remove_driver(id))
	return 1;
}


/* set a callback function to be called to select targets to newly free drivers */
void wte_set_cruising_callback(void* p, unsigned int (*cb)(unsigned int node_id, unsigned int ts)) {
	if(!p) return;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	w->set_cruising_callback([cb](wte::index_t n, unsigned int t) { return (wte::index_t)cb(n, t); });
	return;
}

/* set a callback function to be called to select targets to newly free drivers */
void wte_set_cruising_callback_with_data(void* p, unsigned int (*cb)(unsigned int node_id, unsigned int ts, void* user_data), void* user_data) {
	if(!p) return;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	w->set_cruising_callback([cb, user_data](wte::index_t n, unsigned int t) { return (wte::index_t)cb(n, t, user_data); });
	return;
}
	

/* read the road network used for random walk cruising */
int wte_read_road_network(void* p, const char* fn, int csv, int header) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	read_table2 rt(fn);
	if(csv) rt.set_delim(',');
	if(header) rt.read_line();
	TRY_CALL(w, read_road_network(rt))
	return 1;
}

/* get and set the allowed maximum distance drivers travel during random walk cruising (in meters) */
unsigned int wte_get_random_walk_max_dist(const void* p) {
	if(!p) return 0;
	const wte_trips2<wte_trips>* w = (const wte_trips2<wte_trips>*)p;
	return w->get_random_walk_max_dist();
}

void wte_set_random_walk_max_dist(void* p, unsigned int d) {
	if(!p) return;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	w->set_random_walk_max_dist(d);
}

/* get / set the maximum idle time for drivers; after this, they are removed */
unsigned int wte_get_driver_max_idle(const void* p) {
	if(!p) return 0;
	const wte_trips2<wte_trips>* w = (const wte_trips2<wte_trips>*)p;
	return w->driver_max_idle();
}

void wte_set_driver_max_idle(void* p, unsigned int t) {
	if(!p) return;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	w->driver_max_idle() = t;
}

/* remove all drivers who have been idle more than the maximum idle time set by the previous functions */
int wte_prune_drivers(void* p) {
	if(!p) return 0;
	wte_trips2<wte_trips>* w = (wte_trips2<wte_trips>*)p;
	TRY_CALL(w, prune_drivers())
	return 1;
}


