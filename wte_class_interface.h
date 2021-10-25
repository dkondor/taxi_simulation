/*  -*- C++ -*-
 * wte_class_interface.h -- C-style interface for simulation of
 * 	on-demand mobility; this can be used from C or from Python
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


#ifndef WTE_CLASS_INTERFACE_H
#define WTE_CLASS_INTERFACE_H

#include "trip.h"

#ifdef __cplusplus
extern "C" {
#endif

/*************************************************************************************************
 * main class: taxi simulator among discrete nodes with travel times loaded from external source *
 *************************************************************************************************/
	void* wte_new();
	void wte_free(void* p);
	
	int wte_load_index(void* p, unsigned int matrix_size, const char* fnbase, const char* fnext, const char* fnbase_sorted, const char* fnext_sorted,
		const char* fnbase_paths, const char* fnext_paths, int preread);
	int wte_load_distances(void* p, const char* fn, int preread);
	
	int wte_process_trip(void* p, const void* t, unsigned int tmax1, int allow_moving, int* res1);
		
	int wte_reset(void* p, int clear_drivers);
	int wte_clear(void* p);
		
	/* add one driver at the given node with the given start time */
	int wte_add_driver(void* p, unsigned int node, unsigned int start_time);
	
	
	/* get results from processing the last trip -- returns a pointer to the last result struct */
	const void* wte_get_last_result(const void* p);
	
	/* wte_trips functions */
	void wte_set_seed(void* p, uint64_t seed);
	
	int wte_read_trips(void* p, const char* fn);
	int wte_read_trips_csv(void* p, const char* fn, const char* compress, unsigned int tmin, unsigned int tmax, int tlimit_in_day,
		unsigned int length_min, unsigned int length_max, unsigned int dist_min, unsigned int dist_max, double speed_min, double speed_max, int clear_trips,
		const char* node_filter_fn, int chicago_format);

	int wte_add_trip(void* p, const void* t);
	void wte_sort_trips(void* p, int reassign_ids);
	int wte_is_sorted(const void* p);
	
	int wte_add_drivers(void* p, uint64_t n, double tavg, int use_end_locations);
		
	int wte_process_next_trip(void* p, unsigned int tmax1, int* res1);
	int wte_is_end(const void* p);
	
	
	/* helpers to convert trips to / from python objects */
	const void* wte_get_trip(const void* p, uint64_t idx);
	uint64_t wte_ntrips(const void* p);
	
	/* get a text representation of the last error that occurred (text of exception thrown by C++ code) */
	char* wte_get_error(void* p);
	
	/* Set if C++ exceptions should be handled by storing the error message in a way that
	 * can be retrieved by wte_get_error() and returning 0.
	 * If unset, C++ exceptions are not catched; this can be useful e.g. if running from a debugger. */
	void wte_set_handle_exceptions(void* p, int should_handle_exceptions);
	
	/* Get if C++ exceptions are handled internally. */
	int wte_get_handle_exceptions(const void* p);
	
	
	uint64_t wte_get_trip_idx(const void* p);
	
	/* helper functions to get info about the drivers; this returns the number of active (non-removed) drivers */
	unsigned int wte_get_total_drivers(const void* p);
	/* this returns the number of total driver IDs */
	unsigned int wte_get_total_driver_ids(const void* p);
	/* get info about one driver -- returns a reference to a driver_summary struct */
	const void* wte_get_driver_info(void* p, unsigned int d);
	
	enum wte_cruising_strategy {
			WTE_CRUISING_STRATEGY_NONE = 0, /* no cruising, drivers always stay at the node where they finished their trip */
			WTE_CRUISING_STRATEGY_DRIVER_RANDOM_NODE = 1, /* when finishing their trip, drivers select a random node to move to;
				this can be either completely random (not recommended), or based on a distribution
				that is set by set_cruising_target_dist() */
			WTE_CRUISING_STRATEGY_CALLBACK = 2, /* a callback function is provided by the user that is called whenever a driver
				finishes a trip -- use wte_set_cruising_callback() for this */
			WTE_CRUISING_STRATEGY_RANDOM_WALK = 3 /* a random walk is performed on the road network that has to be supplied with
				the wte_read_road_network() function */
		};
	
	enum wte_cruising_strategy wte_get_cruising_strategy(const void* p);
	void wte_set_cruising_strategy(void* p, enum wte_cruising_strategy strategy);
	
	/* set the cruising target distribution for a given hour (and possibly later hours) in a day */
	int wte_set_cruising_target_dist(void* p, unsigned int hour, unsigned int* nodes, double* weights, size_t len);
	int wte_read_cruising_target_dist(void* p, unsigned int hour, const char* fn);
	
	/* set the cruising target distribution for multiple days */
	int wte_read_cruising_target_dist_mult(void* p, const char* fn);
	
	/* filtering and copying */
	/* filter the trips in the current instance */
	void wte_filter_trips(void* p, const uint64_t* ids, size_t len);
	/* make a copy with filtering the trips */
	void* wte_copy_trips_filter(void* p, const uint64_t* ids, size_t len);
	
	
	
	/****************************************************
	 * get the list of idle and moving drivers 
	 ****************************************************/
	
	/* get the number of idle drivers */
	unsigned int wte_n_idle_drivers(const void* p);
	/* get the number of repositioning drivers */
	unsigned int wte_n_moving_drivers(const void* p);

	/* start iteration on idle drivers */
	void wte_idle_drivers_start(void* p);
	/* return if the end has already be reached when iterating on idle drivers (also returns 1 if the iterator is invalid) */
	int wte_idle_drivers_is_end(const void* p);
	/* Advances the iterator for idle drivers; returns 1 if a valid element is available or 0
	 * if the end has been already reached or if the iterator is invalid. */
	int wte_idle_drivers_advance(void* p);
	/* Stores the ID and node ID of the current idle driver (pointed to be the internal iterator) in the given locations.
	 * Returns 0 if the iterator is invalid or the end has already been reached; otherwise returns 1. */
	int wte_idle_drivers_current(void* p, unsigned int* driver_id1, unsigned int* node_id);
	
	/* start iteration on currently moving drivers */
	void wte_moving_drivers_start(void* p);
	/* return if the end has already be reached when iterating on moving drivers (also returns 1 if the iterator is invalid) */
	int wte_moving_drivers_is_end(const void* p);
	/* Advances the iterator for moving drivers; returns 1 if a valid element is available or 0
	 * if the end has been already reached or if the iterator is invalid. */
	bool wte_moving_drivers_advance(void* p);
	/* Stores the ID and node ID of the current moving driver (pointed to be the internal iterator) in the given locations.
	 * Returns 0 if the iterator is invalid or the end has already been reached; otherwise returns 1. */
	int wte_moving_drivers_current(void* p, unsigned int* driver_id1, unsigned int* node_id);
	
	/* start moving a driver toward a destination using the shortest path;
	 * removes the given driver from either the list of idle or currently
	 * moving drivers; returns 0 if driver is not an idle or
	 * currently moving driver */
	int wte_reposition_driver(void* p, unsigned int id, unsigned int target);
	
	/* remove a driver with the given ID; note: only possible if the driver is not assigned to serve a trip,
	 * i.e. they are idle or cruising */
	int wte_remove_driver(void* p, unsigned int id);
	
	/* get / set the maximum idle time for drivers; after this, they are removed */
	unsigned int wte_get_driver_max_idle(const void* p);
	void wte_set_driver_max_idle(void* p, unsigned int t);
	
	/* remove all drivers who have been idle more than the maximum idle time set by the previous functions */
	int wte_prune_drivers(void* p);
	
	/* set a callback function to be called to select targets to newly free drivers */
	void wte_set_cruising_callback(void* p, unsigned int (*cb)(unsigned int node_id, unsigned int ts));
	
	/* set a callback function to be called to select targets to newly free drivers */
	void wte_set_cruising_callback_with_data(void* p, unsigned int (*cb)(unsigned int node_id, unsigned int ts, void* user_data), void* user_data);
	
	/* read the road network used for random walk cruising */
	int wte_read_road_network(void* p, const char* fn, int csv, int header);

	/* get and set the allowed maximum distance drivers travel during random walk cruising
	 * (in meters; zero means unlimited) */
	unsigned int wte_get_random_walk_max_dist(const void* p);
	void wte_set_random_walk_max_dist(void* p, unsigned int d);
		
#ifdef __cplusplus
}
#endif

#endif


