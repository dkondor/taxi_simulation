/*  -*- C++ -*-
 * wte_class.h -- object oriented interface for simulations of on-demand
 * 	mobility
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


#ifndef WTE_CLASS_H
#define WTE_CLASS_H

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <vector>
#include <queue>
#include <random>
#include <algorithm>
#include <utility>

#include <set>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <functional>

#include "ttimes2.h"
#include "spdist.h"
#include "read_table.h"
#include "trip.h"


/* main class doing the analysis */
class wte {
	public:
		/* node IDs are 16-bit */
		typedef uint16_t index_t;
		/* distances are 32-bit (in meters) */
		typedef uint32_t dist_t;
		/* travel times are 16-bit (in seconds) */
		typedef uint16_t ttime_t;
		/* drivers have a unique, 32-bit ID */
		typedef uint32_t driver_id;
		
		/* struct to hold the result of last processed trip */
		struct last_res_t {
			unsigned int tp = 0; /* (original) start time of trip */
			unsigned int np = 0; /* number of trips processed so far (including this one and any rejected trips) */
			size_t id = 0; /* id of last processed trip */
			driver_id driver = 0; /* id of driver assigned to serve the trip */
			unsigned int nreject = 0; /* number of rejected trips */
			unsigned int tw = 0; /* passenger waiting time */
			driver_id ndrivers = 0; /* number of idle drivers remaining */
			unsigned int tt = 0; /* travel time of the current trip */
			unsigned int di = 0; /* driver idle time (since finishing the last trip) */
			unsigned int trip_dist = 0; /* distance of the trip (only if we have a spatial index) */
			unsigned int driver_dist = 0; /* empty distance (driver going to pick up) */
			index_t driver_node = 0; /* node where the driver was when assigning the trip (can be different
				from the last node where the driver ended their trip if cruising is enable) */
			unsigned int driver_extra_time = 0; /* extra time for the driver before moving to the passenger
				(needed if cruising is enabled and the driver is between two nodes when assigned) */
		};
		
		enum class cruising_strategy {
			none = 0, /* no cruising, drivers always stay at the node where they finished their trip */
			driver_random_node = 1, /* when finishing their trip, drivers select a random node to move to;
				this can be either completely random (not recommended), or based on a distribution
				that is set by set_cruising_target_dist() */
			callback = 2, /* a callback function is provided by the user that is called whenever a driver
				finishes a trip */
			random_walk = 3 /* a random walk is performed on the road network that has to be supplied with
				the read_network() function */
		};
		
		struct driver_summary {
			unsigned int trips_served = 0; /* total number of trips served */
			unsigned int start_time = 0; /* time of the first trip */
			unsigned int end_time = 0; /* end time of the last trip */
			unsigned int total_trip_time = 0; /* total time spent serving passenger trips */
			unsigned int total_empty_time = 0; /* total time spent empty (not including since the end of the last trip and also the time before the start of the first trip) */
			unsigned int total_trip_distance = 0; /* total distance spent serving passenger trips */
			unsigned int total_empty_distance = 0; /* total distance spent driving empty (only between the first and last trip, not including before starting the first trip and after ending the last trip) */
			unsigned int last_empty_distance = 0; /* total empty distance traveled since the end of last trip (while cruising; this is not included in the previous total) */
			enum class state {
				OCCUPIED = 0, /* driver is serving a trip (or is traveling to serve a trip) */
				ADDED = 1, /* initial state of a driver before becomes available */
				IDLE = 2, /* driver is idle (at a node, not moving) */
				MOVING = 3, /* driver is repositioning / cruising */
				REMOVED = 4 /* driver was removed, ID is not valid anymore */
			};
			state st = state::ADDED;
		};
		
	protected:	
		std::shared_ptr<tt_path_index<index_t, tt_sorted_index<index_t> > > ttindex;
		std::shared_ptr<sp_index<dist_t> > spindex;
		
		/* currently available drivers (they are staying at a node, being idle) */
		std::unordered_map<index_t, std::unordered_set<driver_id> > idle_drivers;
		std::unordered_map<driver_id, index_t> drivers_nodes; /* reverse */
		/* iterator that is used when querying idle drivers -- first element denotes if this
		 * iterator is valid (if it is false, the iterator should not be dereferenced) */
		std::pair<bool, std::unordered_map<driver_id, index_t>::const_iterator> driver_it;
		
		/* a driver currently cruising (with a path) */
		struct moving_driver {
			driver_id id;
			std::vector<index_t> path; /* the path the driver is taking */
			std::vector<unsigned int> ts; /* the timestamp for each node */
			size_t i; /* current node: the driver is between path[i] and path[i+1] 
				note: i + 1 < path.size() always */
			/* comparison is by the time to reach the next node */
			bool operator < (const struct moving_driver& b) const {
				bool x = ts[i+1] < b.ts[b.i+1];
				bool y = ts[i+1] == b.ts[b.i+1];
				return x || (y && id < b.id);
			}
			explicit moving_driver(driver_id id_):id(id_),i(0UL) { }
		};
		
		/* list of all moving bikes */
		std::unordered_map<driver_id, moving_driver> moving_drivers;
		/* iterator that is used when querying moving drivers -- first element denotes if this
		 * iterator is valid (if it is false, the iterator should not be dereferenced) */
		std::pair<bool, std::unordered_map<driver_id, moving_driver>::const_iterator> moving_driver_it;
		struct moving_driver_comparer {
			const wte& parent;
			explicit moving_driver_comparer(const wte& p) : parent(p) { }
			bool operator () (driver_id b1, driver_id b2) const {
				if(b1 == b2) return false; /* optimization for the case of finding the exact driver */
				return parent.moving_drivers.at(b1) < parent.moving_drivers.at(b2);
			}
			typedef void is_transparent;
		};
		friend class moving_driver_comparer;
		/* queue: moving drivers are ordered by the time of the next node in their trip */
		std::set<driver_id, moving_driver_comparer> moving_driver_queue;
		/* index into moving drivers by node ID */
		std::unordered_map<index_t, std::unordered_set<driver_id> > moving_drivers_next;
		
		/* update the queue of moving drivers, taking into account all position changes up to ts */
		void update_moving_drivers(unsigned int ts);
		
		/* remove a moving driver -- returns the current target node of the driver (before removing) */
		index_t remove_moving_driver(driver_id d);
		
		/* update a moving driver that is taking part of a random walk -- it
		 * is assumed that the driver is already part of moving drivers and path[0] 
		 * and ts[0] contain the current node and time */
		void update_moving_driver_random_walk(moving_driver& newmb);
		
		/* queue for occupied bikes */
		struct occupied_driver {
			unsigned int ts;
			driver_id id;
			index_t n;
			
			bool operator < (const occupied_driver& x) const {
				/* note: comparison is reversed for use in queue */
				return ts > x.ts;
			}
		};
		std::priority_queue<occupied_driver> occupied_drivers;
		
		/* summary information about drivers */
		std::vector<driver_summary> drivers_summary;
		
		driver_id ndrivers = 0; /* number of drivers */
		driver_id driver_id_next = 0; /* next available driver id */
		
		unsigned int tp = 0; /* current simulation time */
		unsigned int np = 0; /* number of trips processed (including rejected trips) */
		unsigned int nreject = 0; /* number of trips rejected */
		cruising_strategy strategy = cruising_strategy::none;
		// std::shared_ptr<std::discrete_distribution<index_t> > cruising_dist;
		/* cruising target distributions, stored by minimum hour */
		std::map<uint8_t, std::shared_ptr<std::discrete_distribution<index_t> >, std::greater<uint8_t> > cruising_dists;
		std::mt19937_64 rng;
		
		/* process drivers who become available before the current time (tp) */
		void process_drivers();
		
		std::vector<ttime_t> moving_driver_dist_tmp;
		/* start moving a driver toward a destination using the shortest path;
		 * removes the given driver from either the list of idle or currently
		 * moving drivers; throws an exception if driver is not an idle or
		 * currently moving driver */
		void add_moving_driver(driver_id id, index_t target);
		/* start moving a driver toward a destination using the shortest path;
		 * this function assumes that the driver is NOT present in either
		 * idle_drivers or moving_drivers; the caller must supply the driver's
		 * current location in the start (and optionally dummy_start)
		 * parameters -- it is required that target != start
		 * tp1 gives the time that this drivers starts moving; this can be
		 * < tp (if processing a driver who finished a trip earlier), but in
		 * this case, the caller must call update_moving_drivers() after this
		 * to ensure that the location is updated if necessary */
		void add_moving_driver_internal(driver_id id, index_t target, index_t start, index_t dummy_start,
			unsigned int delay, unsigned int dummy_delay, unsigned int tp1);
		/* adds an idle driver (checking that it was not present before) */
		void add_idle_driver(driver_id id, index_t n);
		
		
		
		/* search functionality -- these functions find possible drivers
		 * to serve a trip, and call the given callback function with the
		 * results; the callback should take a pair of driver ID and waiting
		 * time, and return true if the iteration should continue, or false
		 * if it should stop;
		 * note: it is possible that the callback is not called at all
		 * if no suitable drivers were found */
		
		/* search among idle drivers; results are returned ordered by waiting time */
		void search_idle_drivers(const trip& t, unsigned int tmax, const std::function<bool(std::pair<driver_id, unsigned int>)>& cb);
		
		/* serach among moving drivers; results are not ordered, but an extra
		 * parameter is provided with the current minimum waiting time (any later
		 * result will have a higher waiting time than this) */
		void search_moving_drivers(const trip& t, unsigned int tmax, const std::function<bool(std::pair<driver_id, unsigned int>, unsigned int)>& cb);
		
		/* process a matched driver -- trip pair */
		void process_matched_trip(const trip& t, driver_id d, unsigned int tw, bool have_tw, last_res_t& last_res1);
		
		/* process a trip (FCFS) -- internal function with the option to process a trip from the "past" */
		bool process_trip_internal(const trip& t, unsigned int tmax1 = 0, bool allow_moving = true, bool allow_old = false);
		
		last_res_t last_res;
		
		size_t rng_seed = 0;
		
		std::function<index_t(index_t, unsigned int)> cruising_cb;
		
		/* road network (used optionally if cruising strategy is random walk */
		std::unordered_map<index_t, std::unordered_set<index_t> > road_network;
		unsigned int random_walk_max_dist = 0;
		
		/* if a driver is idle (i.e. without passenger, thus including
		 * cruising and random walk) for at least this time, they are erased */
		unsigned int driver_max_idle_time = 0;
		
		/* Helper function to iterate over all elements in a container,
		 * calling the given function. It is allowed to erase the element
		 * for which the function is called. The supplied function should
		 * return a bool indicating if iteration should continue.
		 * The return value is true if it was iterated over all elements. */
		template<class C, class F>
		bool safe_iter(C& cont, F&& f) {
			bool ret = true;
			auto it = cont.begin();
			while(it != cont.end()) {
				auto it2 = it;
				++it;
				ret = f(*it2);
				if(!ret) break;
			}
			return ret;
		}
		
		
	public:
		wte() : driver_it{false, {}}, moving_driver_it{false, {}}, moving_driver_queue(moving_driver_comparer(*this)) {
			rng_seed = time(0);
			rng.seed(rng_seed);
		}
		/* copy constructor: indexes are shared between copied instances */
		wte(const wte& w) : ttindex(w.ttindex), spindex(w.spindex), driver_it{false, {}}, moving_driver_it{false, {}},
			moving_driver_queue(moving_driver_comparer(*this)), strategy(w.strategy), cruising_dists(w.cruising_dists), rng(w.rng),
			cruising_cb(w.cruising_cb), road_network(w.road_network), random_walk_max_dist(w.random_walk_max_dist),
			driver_max_idle_time(w.driver_max_idle_time) { }
		
		void load_index(unsigned int matrix_size, const char* fnbase, const char* fnext, const char* fnbase_sorted = 0,
			const char* fnext_sorted = 0, const char* fnbase_path = 0, const char* fnext_path = 0, bool preread = false);
		void load_distances(const char* fn, bool preread = false);
		
		/* process one trip (as provided) 
		 * return true if trip is successfully served, false if trip is rejected 
		 * throws exception if trip start time is earlier than last processed trip start time 
		 * tmax1 is the maximum acceptable waiting time for this passenger */
		bool process_trip(const trip& t, unsigned int tmax1 = 0, bool allow_moving = true) {
			return process_trip_internal(t, tmax1, allow_moving, false);
		}
		bool process_trip(const trip* t, unsigned int tmax1 = 0, bool allow_moving = true) {
			return process_trip(*t, tmax1, allow_moving);
		}
		
		const last_res_t& get_last_res() const { return last_res; }
		
		/* remove all drivers and reset all variables to zero */
		void reset(bool clear_drivers = false);
		
		/* add one driver at the given node with the given start time */
		void add_driver(unsigned int node, unsigned int start_time);
		
		/* remove a driver -- only works if the driver is not assigned to a trip
		 * (i.e. is idle or cruising) */
		void remove_driver(driver_id id);
		
		/* get the number of total drivers */
		driver_id n_total_drivers() const { return ndrivers; }
		/* get the number of idle drivers */
		driver_id n_idle_drivers() const { return idle_drivers.size(); }
		/* get the number of repositioning drivers */
		driver_id n_moving_drivers() const { return moving_drivers.size(); }
		/* get the total number of available drivers */
		driver_id n_available_drivers() const { return n_idle_drivers() + n_moving_drivers(); }
		/* get the total number of drivers ever created (i.e. the largest driver ID in use) */
		driver_id n_total_drivers_added() const { return driver_id_next; }
		
		/* Iterate over idle and moving drivers; the below functions start an iteration,
		 * advance the (internally-stored) iterators, and return the ID and node of drivers.
		 * Note that any function that modifies the internal state (such as serving a trip
		 * or advancing the time) will invalidate these iterators.
		 */
		
		/* start iteration on idle drivers */
		void idle_drivers_start() { driver_it = { true, drivers_nodes.cbegin() }; }
		/* return if the end has already be reached when iterating on idle drivers (also returns true if the iterator is invalid) */
		bool idle_drivers_is_end() const { return !driver_it.first || (driver_it.second == drivers_nodes.cend()); }
		/* Advances the iterator for idle drivers; returns true if a valid element is available or false
		 * if the end has been already reached. Throws exception if the iterator is invalid. */
		bool idle_drivers_advance() {
			if(!driver_it.first) throw std::runtime_error("wte::idle_drivers_next(): trying to access invalidated iterator!\n");
			if(driver_it.second == drivers_nodes.cend()) return false;
			++driver_it.second;
			return driver_it.second != drivers_nodes.cend();
		}
		/* returns the current idle driver pointed to by the internal iterator (throws exception if the iterators is invalid) */
		std::pair<driver_id, index_t> idle_drivers_current() const {
			if(!driver_it.first)
				throw std::runtime_error("wte::idle_drivers_current(): trying to access invalidated iterator!\n");
			if(driver_it.second == drivers_nodes.cend())
				throw std::runtime_error("wte::idle_drivers_current(): already reached the end!\n");
			return *driver_it.second;
		}
		
		/* start iteration on currently moving drivers */
		void moving_drivers_start() { moving_driver_it = { true, moving_drivers.cbegin() }; }
		/* return if the end has already be reached when iterating on moving drivers (also returns true if the iterator is invalid) */
		bool moving_drivers_is_end() const { return !moving_driver_it.first || (moving_driver_it.second == moving_drivers.cend()); }
		/* Advances the iterator for moving drivers; returns true if a valid element is available or false
		 * if the end has been already reached. Throws exception if the iterator is invalid. */
		bool moving_drivers_advance() {
			if(!moving_driver_it.first)
				throw std::runtime_error("wte::moving_drivers_next(): trying to access invalidated iterator!\n");
			if(moving_driver_it.second == moving_drivers.cend()) return false;
			++moving_driver_it.second;
			return moving_driver_it.second != moving_drivers.cend();
		}
		/* returns the current moving driver pointed to by the internal iterator (throws exception if the iterators is invalid) */
		std::pair<driver_id, index_t> moving_drivers_current() const {
			if(!moving_driver_it.first)
				throw std::runtime_error("wte::moving_drivers_current(): trying to access invalidated iterator!\n");
			if(moving_driver_it.second == moving_drivers.cend())
				throw std::runtime_error("wte::moving_drivers_current(): already reached the end!\n");
			const moving_driver& d = moving_driver_it.second->second;
			return { moving_driver_it.second->first, d.path[d.i + 1] };
		}
		
		
		/* start moving a driver toward a destination using the shortest path;
		 * removes the given driver from either the list of idle or currently
		 * moving drivers; throws an exception if driver is not an idle or
		 * currently moving driver */
		void reposition_driver(driver_id id, index_t target) { add_moving_driver(id, target); }
		
		
		const driver_summary& get_driver_summary(driver_id d) const { return drivers_summary.at(d); }
		
		/* get / set the cruising strategy used by drivers */
		cruising_strategy& driver_strategy() { return strategy; }
		const cruising_strategy& driver_strategy() const { return strategy; }
		
		/* set the target distribution of cruising drivers */
		void set_cruising_target_dist(uint8_t hour, const std::vector<double>& freqs) {
			if(freqs.size() > ttindex->get_matrix_size()) throw std::runtime_error("wte::set_cruising_target_dist(): too many targets provided!\n");
			cruising_dists[hour].reset(new std::discrete_distribution<index_t>(freqs.begin(), freqs.end()));
		}
		template<class It, class Sent>
		void set_cruising_target_dist(uint8_t hour, It&& it, const Sent& sent) {
			std::vector<double> w(ttindex->get_matrix_size());
			for(; it != sent; ++it) {
				const auto& x = *it;
				w.at(x.first) = x.second;
			}
			set_cruising_target_dist(hour, w);
		}
		
		/* read a target distribution for a specific hour */
		void read_cruising_target_dist(uint8_t hour, read_table2& rt);
		void read_cruising_target_dist(uint8_t hour, read_table2&& rt) { read_cruising_target_dist(hour, rt); }
		void read_cruising_target_dist(uint8_t hour, const char* fn) { read_cruising_target_dist(hour, read_table2(fn)); }
		
		/* read a combined distribution; the first column is the start hour */
		void read_cruising_target_dist_mult(read_table2& rt);
		void read_cruising_target_dist_mult(read_table2&& rt) { read_cruising_target_dist_mult(rt); }
		void read_cruising_target_dist_mult(const char* fn) { read_cruising_target_dist_mult(read_table2(fn)); }
		
		/* set a callback function for cruising drivers
		 * the supplied function gets the node ID where the driver is and the current timestamp;
		 * it should return a target node ID */
		void set_cruising_callback(std::function<index_t(index_t, unsigned int)>&& fn) { cruising_cb = std::move(fn); }
		void set_cruising_callback(std::function<const index_t(index_t, unsigned int)>& fn) { cruising_cb = fn; }
		
		/* read the road network for random walk cruising strategy */
		void read_road_network(read_table2& rt);
		void read_road_network(read_table2&& rt) { read_road_network(rt); }
		void read_road_network(const char* fn) { read_road_network(read_table2(fn)); }
		
		unsigned int get_random_walk_max_dist() const { return random_walk_max_dist; }
		void set_random_walk_max_dist(unsigned int d) { random_walk_max_dist = d; }
		
		/* get / set the maximum idle time for drivers */
		unsigned int& driver_max_idle() { return driver_max_idle_time; }
		unsigned int driver_max_idle() const { return driver_max_idle_time; }
		
		/* prune idle drivers (i.e. get rid of all drivers who have been idle more than driver_max_idle_time) */
		void prune_drivers();
		
		/* set the seed for the random number generator used internally */
		void seed(uint64_t s) { rng.seed(s); rng_seed = s; }
};



/* class encapsulating the index and a list of trips together */
class wte_trips : public wte {
	protected:
		std::vector<trip> trips;
		std::uniform_real_distribution<double> prdist;
		size_t i;
		bool sorted;
		
		void check_trips_last_sorted() {
			if(sorted) {
				size_t n = trips.size();
				if(n > 1 && trips[n-1].start_ts < trips[n-2].start_ts) sorted = false;
			}
		}
		
	public:
		wte_trips():wte(),prdist(0.0,1.0),i(0UL),sorted(true),pwait(1.0,600.0),pr(1.0) { }
		/* copy constructor makes a copy of the trips -- if this is not needed, use the alternative with an empty filter */
		wte_trips(const wte_trips& w) : wte(w), trips(w.trips) { }
		/* "copy" constructor that filters trips */
		wte_trips(const wte_trips& w, const std::unordered_set<uint64_t>& trip_filter) : wte(w) { copy_filtered_trips(w.trips, trip_filter); }
		/* probability distribution of users' maximum acceptable wait times */
		std::gamma_distribution<double> pwait; /* (alpha,beta*60.0) -- note: beta is given in minutes */
		/* probability of using any of the trips */
		double pr;
		double get_random_wait_time() { return pwait(rng); }
		
		/* filter to use when reading trips (zero means no filter for all parameters,
		 * except length_min, where zero length trips are always filtered out) */
		struct read_csv_filter {
			unsigned int tmin = 0; /* leave out trips starting before this time */
			unsigned int tmax = 0; /* leave out trips starting after this time */
			bool tlimit_in_day = false; /* set to true if limits should be interpreted as time within a day */
			unsigned int length_min = 0; /* leave out trips that have this length or less */
			unsigned int length_max = 0; /* leave out trips longer than this */
			unsigned int dist_min = 0; /* leave out trips with less distance */
			unsigned int dist_max = 0; /* leave out trips with longer distance than this */
			double speed_min = 0; /* leave out trips with a slower average speed than this */
			double speed_max = 0; /* leave out trips with a faster average speed than this */
			const char* nodes_file = nullptr; /* if given, read a list of nodes from this file, filter trips to only include those where the start and end is in this */
		};
		
		/* read a list of trips to use */
		void read_trips(read_table2& rt);
		void read_trips(read_table2&& rt) { return read_trips(rt); }
		void read_trips(const char* fn) { return read_trips(read_table2(fn)); }
		void read_trips(FILE* f) { return read_trips(read_table2(f)); }
		/* read trips from a csv file (new format, ignore IDs (fill them out based on trip numbers) */
		void read_trips_csv(read_table2& rt, const read_csv_filter& filt, bool clear_trips, bool chicago_format = false);
		void read_trips_csv(read_table2&& rt, const read_csv_filter& filt, bool clear_trips, bool chicago_format = false) {
			read_trips_csv(rt, filt, clear_trips, chicago_format);
		}
		void read_trips_csv(FILE* f, const read_csv_filter& filt, bool clear_trips, bool chicago_format = false) {
			read_trips_csv(read_table2(f), filt, clear_trips, chicago_format);
		}
		/* read trips from a csv file, optionally compressed with gzip, bzip2 or xz */
		void read_trips_csv(const char* fn, const read_csv_filter& filt,
			const char* compress = nullptr, bool clear_trips = true, bool chicago_format = false);
		
		
		/* add a trip to the internal list */
		void add_trip(const trip& t) { trips.push_back(t); check_trips_last_sorted(); }
		void add_trip(const trip* t) { add_trip(*t); }
		
		/* sort the trips by time */
		void sort_trips(bool reassign_ids = false);
		bool is_sorted() const { return sorted; }
		
		void clear() { wte::reset(); trips.clear(); i = 0; sorted = true; }
		void reset(bool clear_drivers = false) { wte::reset(clear_drivers); i = 0; }
		
		/* add the given number of drivers, with locations distributed according to trip start / end locations */
		void add_drivers(size_t n, double tavg = -1.0, bool use_end_locations = false);
		
		bool process_next_trip(unsigned int tmax1 = 0);
		bool is_end() const { return i >= trips.size(); }
		
		const trip* get_trip(size_t i) const { return &(trips.at(i)); }
		size_t ntrips() const { return trips.size(); }
		size_t trip_idx() const { return i; }
		
		/* filter trips */
		void copy_filtered_trips(const std::vector<trip>& trips1, const std::unordered_set<uint64_t>& trip_filter);
		void filter_trips(const std::unordered_set<uint64_t>& trip_filter);
};




#endif


