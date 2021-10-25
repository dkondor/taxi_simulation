/*
 * wte_class.cpp -- object oriented implementation for simulations of
 * 	on-demand mobility
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


#include "wte_class.h"
#include "iterator_zip.h"
#include <unordered_set>
#include <iterator>
#include <string>
#include <sstream>

static bool read_trip(read_table2& rt, trip& t) { return rt.read(t.id,t.start_ts,t.start_node,t.end_ts,t.end_node); }

static auto sort_trips_by_id() {
	return [](const trip& t1, const trip& t2) {
		if(t1.id == t2.id) throw std::runtime_error("Duplicate trip IDs when sorting trips!\n");
		return t1.id < t2.id;
	};
}
static auto sort_tips_by_time() {
	return [](const trip& t1, const trip& t2) {
		if(t1.id == t2.id) throw std::runtime_error("Duplicate trip IDs when sorting trips!\n");
		return (t1.start_ts < t2.start_ts || (t1.start_ts == t2.start_ts && t1.id < t2.id));
	};
}



void wte::load_index(unsigned int matrix_size, const char* fnbase, const char* fnext, const char* fnbase_sorted,
		const char* fnext_sorted, const char* fnbase_path, const char* fnext_path, bool preread) {
	ttindex.reset();
	auto tmp = std::make_unique<tt_path_index<index_t, tt_sorted_index<index_t> > >(matrix_size);
	if(!tmp) throw std::runtime_error("wte::load_index(): cannot allocate memory!\n");
	tmp->open_files(fnbase, fnext, true);
	if(fnbase_sorted) tmp->open_sorted_files(fnbase_sorted, fnext_sorted, true);
	if(fnbase_path) tmp->open_ancestor_files(fnbase_path, fnext_path);
	ttindex = std::move(tmp);
	if(preread) ttindex->prefault();
}

void wte::load_distances(const char* fn, bool preread) {
	spindex.reset();
	auto tmp = std::make_unique<sp_index<dist_t> >();
	if(!tmp) throw std::runtime_error("wte::load_distances(): cannot allocate memory!\n");
	if(!tmp->open(fn)) {
		std::ostringstream strs;
		strs << "wte::load_distances(): cannot open file " << fn << "!\n";
		throw std::runtime_error(strs.str());
	}
	if(ttindex && ttindex->get_matrix_size() != tmp->get_matrix_size()) {
		throw std::runtime_error("Matrix size for distances differs from travel times!\n");
	}
	spindex = std::move(tmp);
	if(preread) spindex->prefault();
}

void wte::add_idle_driver(driver_id id, index_t n) {
	driver_it.first = false;
	if(!idle_drivers[n].insert(id).second || !drivers_nodes.insert(std::make_pair(id, n)).second)
		throw std::runtime_error("wte::add_idle_driver(): duplicated driver!\n");
	drivers_summary.at(id).st = driver_summary::state::IDLE;
}

void wte::process_drivers() {
	driver_it.first = false;
	moving_driver_it.first = false;
	/* process drivers who become available */
	while(!occupied_drivers.empty() && occupied_drivers.top().ts <= tp) {
		const auto& d = occupied_drivers.top();
		switch(strategy) {
			case cruising_strategy::driver_random_node:
			{
				index_t target;
				if(!cruising_dists.empty()) {
					uint8_t h = (tp % 86400) / 3600;
					auto it = cruising_dists.lower_bound(h);
					if(it == cruising_dists.end())
						throw std::runtime_error("wte::process_drivers(): Incomplete cruising target distribution!\n");
					target = it->second->operator()(rng);
				}
				else {
					std::uniform_int_distribution<index_t> tmp(0, ttindex->get_matrix_size() - 1);
					target = tmp(rng);
				}
				/* note: if we happen to select the node the driver is already at,
				 * no need to move them */
				if(target == d.n) add_idle_driver(d.id, d.n);
				else add_moving_driver_internal(d.id, target, d.n, 0, 0, 0, d.ts);
				break;
			}
			case cruising_strategy::none:
				add_idle_driver(d.id, d.n);
				break;
			case cruising_strategy::callback:
			{
				index_t target = cruising_cb(d.n, d.ts);
				if(target == d.n) add_idle_driver(d.id, d.n);
				else add_moving_driver_internal(d.id, target, d.n, 0, 0, 0, d.ts);
				break;
			}
			case cruising_strategy::random_walk:
			{
				auto res = moving_drivers.emplace(d.id, d.id);
				if(!res.second) throw std::runtime_error("wte::add_moving_driver_internal(): driver ID already present among moving drivers!\n");
				
				auto& newmb = res.first->second;
				newmb.path.resize(2);
				newmb.ts.resize(2);
				newmb.path[0] = d.n;
				newmb.ts[0] = d.ts;
				update_moving_driver_random_walk(newmb);
				drivers_summary.at(d.id).st = driver_summary::state::MOVING;
				break;
			}
			default:
				throw std::runtime_error("wte::process_drivers(): Invalid cruising strategy!\n");
		}
		occupied_drivers.pop();
	}
	update_moving_drivers(tp);
}

wte::index_t wte::remove_moving_driver(driver_id d) {
	moving_driver_it.first = false;
	auto it = moving_drivers.find(d);
	if(it == moving_drivers.end()) throw std::runtime_error("wte::remove_moving_driver(): driver not found!\n");
	const auto& md = it->second;
	index_t next_node = md.path[md.i + 1];
	if(moving_drivers_next.at(next_node).erase(d) != 1)
		throw std::runtime_error("wte::remove_moving_driver(): driver not found!\n");
	if(!moving_driver_queue.erase(d))
		throw std::runtime_error("wte::remove_moving_driver(): driver not found!\n");
	moving_drivers.erase(it);
	return next_node;
}

void wte::update_moving_drivers(unsigned int ts) {
	moving_driver_it.first = false;
	while(!moving_driver_queue.empty()) {
		driver_id id = *(moving_driver_queue.cbegin());
		if(driver_max_idle_time > 0) {
			const auto& s = drivers_summary.at(id);
			if(s.end_time > s.start_time && s.end_time < tp && tp - s.end_time > driver_max_idle_time) {
				remove_driver(id); // note: this will remove the driver from the moving_driver_queue as well
				continue;
			}
		}
		
		moving_driver& b = moving_drivers.at(id);
		if(b.ts[b.i + 1] >= ts) break;
		
		moving_driver_queue.erase(moving_driver_queue.begin());
		index_t next_node = b.path[b.i + 1];
		
		if(moving_drivers_next.at(next_node).erase(id) != 1)
			throw std::runtime_error("wte::update_moving_drivers(): driver not found!\n");
		
		for(; b.i + 1 < b.path.size() && b.ts[b.i + 1] < ts; b.i++)
			if(spindex) drivers_summary.at(id).last_empty_distance +=
				spindex->get_dist_ix(b.path[b.i], b.path[b.i + 1]);
		
		if(b.i + 1 == b.path.size()) {
			/* this driver is at the end of their path */
			if(strategy == cruising_strategy::random_walk && (!random_walk_max_dist ||
					drivers_summary.at(id).last_empty_distance < random_walk_max_dist)) {
				/* continue the random walk */
				if(b.i != 1) throw std::runtime_error("wte::update_moving_drivers(): inconsistent random walk driver!\n");
				b.path[0] = b.path[1];
				b.ts[0] = b.ts[1];
				b.i = 0;
				update_moving_driver_random_walk(b);
			}
			else {
				add_idle_driver(id, b.path[b.i]);
				moving_drivers.erase(id);
			}
		}
		else {
			/* move this driver to the next node */
			next_node = b.path[b.i + 1];
			moving_drivers_next[next_node].insert(id);
			moving_driver_queue.insert(id);
		}
	}
}

void wte::add_moving_driver(driver_id id, index_t target) {
	if(!ttindex) throw std::runtime_error("wte::add_moving_driver(): no travel time index loaded!\n");
	
	unsigned int delay = 0; /* delay if currently moving */
	index_t start = 0;
	index_t dummy_start = 0; /* if delay > 0, we need a dummy start node */
	unsigned int dummy_delay = 0;
	
	auto it = drivers_nodes.find(id);
	if(it != drivers_nodes.end()) {
		/* this is a currently idle driver */
		start = it->second;
		if(start == target) return; /* nothing to be done if the driver is already at the destination */
		if(!idle_drivers.at(start).erase(id))
			throw std::runtime_error("wte::add_moving_driver(): idle driver not found!\n");
		drivers_nodes.erase(it);
	}
	else {
		/* this is a currently moving driver */
		auto it2 = moving_drivers.find(id);
		if(it2 == moving_drivers.end())
			throw std::runtime_error("wte::add_moving_driver(): requested driver ID not found!\n");
		
		auto& mb = it2->second;
		index_t node1 = mb.path[mb.i];
		index_t node2 = mb.path[mb.i + 1];
		unsigned int ts1 = mb.ts[mb.i];
		unsigned int ts2 = mb.ts[mb.i + 1];
		
		if(ts1 > tp || ts2 < tp)
			throw std::runtime_error("wte::add_moving_driver(): inconsistent moving drivers!\n");
		
		/* the start node is always node2 (driver cannot turn back) */
		start = node2;
		if(start == target) {
			/* just need to shorten the current path if the target node is already in it */
			mb.path.resize(mb.i + 2);
			mb.ts.resize(mb.i + 2);
			return;
		}
		dummy_start = node1;
		delay = ts2 - tp;
		dummy_delay = tp - ts1;
		
		/* erase from moving drivers */
		auto qit = moving_driver_queue.find(id);
		if(qit == moving_driver_queue.end())
			throw std::runtime_error("wte::add_moving_driver(): inconsistent moving drivers!\n");
		moving_driver_queue.erase(qit);
		if(!moving_drivers_next.at(node2).erase(id))
			throw std::runtime_error("wte::add_moving_driver(): inconsistent moving drivers!\n");
		moving_drivers.erase(it2);
	}
	
	add_moving_driver_internal(id, target, start, dummy_start, delay, dummy_delay, tp);
}

void wte::update_moving_driver_random_walk(moving_driver& newmb) {
	index_t start = newmb.path[0];
	index_t target;
	{
		auto it0 = road_network.find(start);
		if(it0 == road_network.end()) throw std::runtime_error("wte::update_moving_driver_random_walk(): node not found!\n");
		const auto& neighbors = it0->second;
		if(neighbors.size() == 0) throw std::runtime_error("wte::update_moving_driver_random_walk(): node has no neighbors!\n");
		size_t ix = 0;
		if(neighbors.size() > 1) {
			std::uniform_int_distribution<size_t> tmp(0, neighbors.size() - 1);
			ix = tmp(rng);
		}
		auto it = neighbors.begin();
		std::advance(it, ix);
		target = *it;
	}
	
	newmb.path[1] = target;
	unsigned int tp1 = newmb.ts[0];
	unsigned int hour1 = (tp1 % 86400) / 3600;
	tp1 += ttindex->get_travel_time_id(start, target, (uint8_t)hour1);
	newmb.ts[1] = tp1;
	
	/* update the helper structs */
	moving_driver_queue.insert(newmb.id);
	moving_drivers_next[newmb.path[1]].insert(newmb.id);
}

void wte::add_moving_driver_internal(driver_id id, index_t target, index_t start, index_t dummy_start, unsigned int delay, unsigned int dummy_delay, unsigned int tp1) {
	moving_driver_it.first = false;
	/* find an actual path */
	std::vector<index_t> path;
	std::vector<ttime_t>& dist = moving_driver_dist_tmp;
	{
		unsigned int hour1 = (tp1 % 86400) / 3600;
		if(!ttindex->find_path(start, target, (uint8_t)hour1, path, dist, false) || path.size() < 2) {
			std::ostringstream strs(std::string("wte::add_moving_driver_internal(): error finding a path!\n"), std::ios_base::ate);
			strs << "start: " << start << ", target: " << target << ", hour: " << hour1 << ", path size: " << path.size() << "\n";
			strs << "random seed: " << rng_seed << "\n";
			throw std::runtime_error(strs.str());
		}
	}
	
	/* calculate timestamps along the path */
	auto res = moving_drivers.emplace(id, id);
	if(!res.second) throw std::runtime_error("wte::add_moving_driver_internal(): driver ID already present among moving drivers!\n");
	
	if(delay) {
		path.push_back(dummy_start);
		dist.push_back(0);
	}
	std::reverse(path.begin(), path.end());
	std::reverse(dist.begin(), dist.end());
	
	auto& newmb = res.first->second;
	newmb.path = std::move(path);
	newmb.ts.reserve(newmb.path.size());
	if(delay) {
		newmb.ts.push_back(tp1 - dummy_delay);
		newmb.ts.push_back(tp1 + delay);
	}
	else newmb.ts.push_back(tp1);
	unsigned int total_ttime = 0;
	/* note: first and last element are zero; ith element is the 
	 * distance between the ith and i+1th element in path */
	for(size_t i = delay ? 1 : 0; i + 1 < newmb.path.size(); i++) {
		total_ttime += dist[i];
		newmb.ts.push_back(tp1 + delay + total_ttime);
	}
	dist.clear();
	
	/* update the helper structs */
	moving_driver_queue.insert(id);
	moving_drivers_next[newmb.path[1]].insert(id);
	drivers_summary.at(id).st = driver_summary::state::MOVING;
}



void wte::search_idle_drivers(const trip& t, unsigned int tmax, const std::function<bool(std::pair<driver_id, unsigned int>)>& cb) {
	uint8_t h = (tp % 86400) / 3600;
	const index_t* nodes = ttindex ? ttindex->get_sorted_ids_to(h, t.start_node) : nullptr;
	if(!nodes) throw std::runtime_error("wte::search_idle_drivers(): no travel time index loaded!\n");
	
	unsigned int nnodes = ttindex->get_matrix_size();
	for(unsigned int i = 0; i < nnodes; i++) {
		index_t n = nodes[i];
		unsigned int tt1 = ttindex->get_travel_time_id(n, t.start_node, h);
		if(tp > t.start_ts) tt1 += tp - t.start_ts;
		if(tmax && tt1 > tmax) break;
		auto it = idle_drivers.find(n);
		if(it != idle_drivers.end()) {
			bool cont = safe_iter(it->second, [this, &cb, tt1](driver_id d) {
				if(driver_max_idle_time > 0) {
					/* check if this driver was idle for too long and
					 * erase them in that case */
					const auto& s = drivers_summary.at(d);
					if(s.end_time > s.start_time && s.end_time < tp &&
							tp - s.end_time > driver_max_idle_time) {
						remove_driver(d);
						return true;
					}
				}
				return cb(std::make_pair(d, tt1));
			});
			if(!cont) break;
		}
	}
}

void wte::search_moving_drivers(const trip& t, unsigned int tmax, const std::function<bool(std::pair<driver_id, unsigned int>, unsigned int)>& cb) {
	uint8_t h = (tp % 86400) / 3600;
	const index_t* nodes = ttindex ? ttindex->get_sorted_ids_to(h, t.start_node) : nullptr;
	if(!nodes) throw std::runtime_error("wte::search_moving_drivers(): no travel time index loaded!\n");
	
	unsigned int nnodes = ttindex->get_matrix_size();
	for(unsigned int i = 0; i < nnodes; i++) {
		index_t n = nodes[i];
		unsigned int tt1 = ttindex->get_travel_time_id(n, t.start_node, h);
		if(tp > t.start_ts) tt1 += tp - t.start_ts;
		if(tmax && tt1 > tmax) break;
		
		/* look at only the next node of drivers -- we assume they cannot turn back, only change course at intersections */
		auto it = moving_drivers_next.find(n);
		if(it != moving_drivers_next.end()) {
			bool cont = safe_iter(it->second, [this, &cb, tt1, tmax](driver_id d) {
				if(driver_max_idle_time > 0) {
					/* check if this driver was idle for too long and
					 * erase them in that case */
					const auto& s = drivers_summary.at(d);
					if(s.end_time > s.start_time && s.end_time < tp &&
							tp - s.end_time > driver_max_idle_time) {
						remove_driver(d);
						return true;
					}
				}
				
				const moving_driver& md = moving_drivers.at(d);
				unsigned int tsnext = md.ts[md.i + 1];
				if(tsnext < tp) throw std::runtime_error("wte::search_moving_drivers(): inconsistent moving drivers!\n");
				unsigned int tt2 = tt1 + (tsnext - tp);
				if(tmax && tt2 > tmax) return true;
				return cb(std::make_pair(d, tt2), tt1);
			});
			if(!cont) break;
		}
	}
}


bool wte::process_trip_internal(const trip& t, unsigned int tmax1, bool allow_moving, bool allow_old) {
	if(!ttindex) throw std::runtime_error("wte::process_trip_internal(): no travel time index loaded!\n");
	if(t.start_node >= ttindex->get_matrix_size() || t.end_node >= ttindex->get_matrix_size())
		throw std::runtime_error("wte::process_trip_internal(): Invalid node IDs!\n");
	
	if(t.start_ts < tp) {
		if(!allow_old) throw std::runtime_error("wte::process_trip(): trips not sorted!\n");
	}
	else {
		tp = t.start_ts;
		process_drivers();
	}
	np++;
	
	bool idle_found = false;
	std::pair<driver_id, unsigned int> idle_res;
	bool moving_found = false;
	std::pair<driver_id, unsigned int> moving_res;
	
	search_idle_drivers(t, tmax1, [&idle_found, &idle_res](std::pair<driver_id, unsigned int> res) {
		idle_res = res;
		idle_found = true;
		return false;
	});
	if(allow_moving) search_moving_drivers(t, tmax1,
			[idle_found, idle_res, &moving_found, &moving_res](std::pair<driver_id, unsigned int> res, unsigned int tt1) {
		if(moving_found) {
			if(res.second < moving_res.second) moving_res = res;
		}
		else {
			if(!idle_found || res.second < idle_res.second) {
				moving_res = res;
				moving_found = true;
			}
		}
		/* here, at least one of idle_found or moving_found is true */
		if(moving_found) {
			if(tt1 >= moving_res.second) return false;
			else return true;
		}
		if(idle_found) {
			if(tt1 >= idle_res.second) return false;
			else return true;
		}
		return true; /* should not happen */
	});
	
	if(!(idle_found || moving_found)) {
		nreject++;
		return false;
	}
	
	if(moving_found) moving_driver_it.first = false;
	else driver_it.first = false;
	
	/* found driver, process successful trip */
	driver_id d = moving_found ? moving_res.first : idle_res.first;
	unsigned int tw = moving_found ? moving_res.second : idle_res.second;
	process_matched_trip(t, d, tw, true, last_res);
	
	return true;
}

void wte::process_matched_trip(const trip& t, driver_id d, unsigned int tw, bool have_tw, last_res_t& last_res1) {
	index_t driver_node, last_driver_node;
	bool moving_found = moving_drivers.count(d);
	if(moving_found) {
		const auto& md = moving_drivers.at(d);
		driver_node = md.path[md.i + 1];
		last_driver_node = md.path[md.i];
		moving_driver_it.first = false;
	}
	else {
		driver_node = drivers_nodes.at(d);
		last_driver_node = driver_node;
		driver_it.first = false;
	}
	last_res1.driver_node = driver_node;
	
	/* re-calculate waiting time if not known */
	if(moving_found) {
		const auto& md = moving_drivers.at(d);
		unsigned int ts1 = md.ts[md.i + 1];
		if(ts1 < tp) throw std::runtime_error("wte::process_matched_trip(): inconsistent moving drivers!\n");
		last_res1.driver_extra_time = ts1 - tp;
	}
	else last_res1.driver_extra_time = 0;
	if(!have_tw) {
		unsigned int hour = (tp % 86400) / 3600;
		tw = ttindex->get_travel_time_id(driver_node, t.start_node, (uint8_t)hour);
		tw += (tp - t.start_ts) + last_res1.driver_extra_time;
	}
	
	driver_summary& ds = drivers_summary.at(d);
	/* drivers idle time */
	unsigned int di = ds.trips_served ? (t.start_ts + tw - ds.end_time) : 0;
	ds.end_time = t.end_ts + tw;
	ds.total_trip_time += (t.end_ts - t.start_ts);
	ds.total_empty_time += di;
	if(ds.trips_served) {
		if(spindex) {
			unsigned int empty_distance = ds.last_empty_distance;
			empty_distance += spindex->get_dist_ix(driver_node, t.start_node);
			if(moving_found) empty_distance += spindex->get_dist_ix(last_driver_node, driver_node);
			ds.total_empty_distance += empty_distance;
			last_res1.driver_dist = empty_distance;
			ds.last_empty_distance = 0;
		}
	}
	else ds.start_time = tp + tw;
	if(spindex) {
		unsigned int trip_dist = spindex->get_dist_ix(t.start_node, t.end_node);
		ds.total_trip_distance += trip_dist;
		last_res1.trip_dist = trip_dist;
	}
	ds.trips_served++;
	ds.st = driver_summary::state::OCCUPIED;
	
	/* remove the driver */
	if(moving_found) remove_moving_driver(d);
	else {
		auto it = drivers_nodes.find(d);
		if(it == drivers_nodes.end()) throw std::runtime_error("wte::process_matched_trip(): driver not found!\n");
		driver_node = it->second;
		if(!idle_drivers.at(it->second).erase(d)) throw std::runtime_error("wte::process_matched_trip(): driver not found!\n");
		drivers_nodes.erase(it);
	}
	
	/* add driver at the end of the trip */
	index_t nn = (index_t)t.end_node;
	if(nn >= ttindex->get_matrix_size()) throw std::runtime_error("wte::process_matched_trip(): Invalid trip destination!\n");
	occupied_drivers.push(occupied_driver{t.end_ts + tw, d, nn});
	
	/* "save" results */
	last_res1.tp = tp; /* (original) start time of trip */
	last_res1.np = np; /* number of trips processed so far (including this one and any rejected trips) */
	last_res1.nreject = nreject; /* number of rejected trips */
	last_res1.tw = tw; /* passenger waiting time */
	last_res1.tt = t.end_ts - t.start_ts; /* travel time of the current trip */
	last_res1.di = di; /* driver idle time (since finishing the last trip) */
	last_res1.driver = d; /* id of driver assigned to serve the trip */
	last_res1.ndrivers = drivers_nodes.size() + moving_drivers.size(); /* number of idle drivers remaining */
	last_res1.id = t.id; /* id of last processed trip */
}

void wte::reset(bool clear_drivers) {
	if(clear_drivers) {
		idle_drivers.clear();
		drivers_nodes.clear();
		moving_drivers.clear();
		moving_driver_queue.clear();
		moving_drivers_next.clear();
		drivers_summary.clear();
		occupied_drivers = std::priority_queue<occupied_driver>();
		ndrivers = 0;
		driver_id_next = 0;
	}
	else {
		/* process remaining queues -- make all drivers idle */
		tp = std::numeric_limits<unsigned int>::max();
		process_drivers();
		/* reset the number of trips served by each driver */
		for(driver_id d = 0; d < driver_id_next; d++) if(drivers_summary[d].st != driver_summary::state::REMOVED) {
			auto st = drivers_summary[d].st;
			drivers_summary[d] = driver_summary();
			drivers_summary[d].st = st;
		}
	}
	tp = 0;
	np = 0;
	nreject = 0;
	moving_driver_it.first = false;
	driver_it.first = false;
}

void wte::add_driver(unsigned int node, unsigned int start_time) {
	index_t nn = (index_t)node;
	if(nn >= ttindex->get_matrix_size()) throw std::runtime_error("wte::add_driver(): Invalid node!\n");
	driver_id id = driver_id_next;
	occupied_drivers.push(occupied_driver{start_time, id, nn});
	drivers_summary.emplace_back();
	ndrivers++;
	driver_id_next++;
}

void wte::remove_driver(driver_id id) {
	auto it = drivers_nodes.find(id);
	if(it != drivers_nodes.end()) {
		if(!idle_drivers.at(it->second).erase(id))
			throw std::runtime_error("wte::remove_driver(): idle driver not found!\n");
		drivers_nodes.erase(it);
	}
	else {
		/* this is a currently moving driver */
		auto it2 = moving_drivers.find(id);
		if(it2 == moving_drivers.end())
			throw std::runtime_error("wte::remove_driver(): requested driver ID not found!\n");
		
		auto& mb = it2->second;
		index_t node2 = mb.path[mb.i + 1];
		
		/* erase from moving drivers */
		auto qit = moving_driver_queue.find(id);
		if(qit == moving_driver_queue.end())
			throw std::runtime_error("wte::remove_driver(): inconsistent moving drivers!\n");
		moving_driver_queue.erase(qit);
		if(!moving_drivers_next.at(node2).erase(id))
			throw std::runtime_error("wte::remove_driver(): inconsistent moving drivers!\n");
		moving_drivers.erase(it2);
	}
	
	/* erase from driver summary */
	drivers_summary.at(id).st = driver_summary::state::REMOVED;
	if(!ndrivers) throw std::runtime_error("wte::remove_driver(): invalid driver count!\n");
	ndrivers--;
}

void wte::read_cruising_target_dist(uint8_t hour, read_table2& rt) {
	std::vector<std::pair<index_t, double> > cd;
	while(rt.read_line()) {
		std::pair<index_t, double> tmp;
		if(!rt.read(tmp.first, tmp.second)) break;
		cd.push_back(tmp);
	}
	if(rt.get_last_error() != T_EOF) {
		std::string ex = rt.exception_string("Error reading cruising target distribution:\n");
		throw std::runtime_error(ex);
	}
	set_cruising_target_dist(hour, cd.begin(), cd.end());
}

void wte::read_cruising_target_dist_mult(read_table2& rt) {
	cruising_dists.clear();
	std::unordered_map<uint8_t, std::vector<std::pair<index_t, double> > > cd;
	while(rt.read_line()) {
		unsigned int hour;
		index_t n;
		double w;
		if(!rt.read(read_bounds(hour, 0U, 23U), n, w)) break;
		cd[(uint8_t)hour].push_back(std::pair<index_t, double>(n, w));
	}
	if(rt.get_last_error() != T_EOF) {
		std::string ex = rt.exception_string("Error reading cruising target distribution:\n");
		throw std::runtime_error(ex);
	}
	for(const auto& x : cd) {
		set_cruising_target_dist(x.first, x.second.begin(), x.second.end());
	}
}


void wte::read_road_network(read_table2& rt) {
	road_network.clear();
	while(rt.read_line()) {
		index_t n1, n2;
		if(!rt.read(n1, n2)) break;
		road_network[n1].insert(n2);
	}
	if(rt.get_last_error() != T_EOF) throw std::runtime_error(rt.exception_string("wte::read_road_network(): error reading input:\n"));
}


void wte_trips::read_trips(read_table2& rt) {
	trips.clear();
	i = 0;
	sorted = true;
	while(rt.read_line()) {
		trip t;
		if(!read_trip(rt,t)) break;
		trips.push_back(t);
		check_trips_last_sorted();
	}
	if(rt.get_last_error() != T_EOF) {
		std::string ex_str = rt.exception_string("wte_trips::read_trips(): Error reading trips:\n");
		throw std::runtime_error(ex_str);
	}
	if(!sorted) sort_trips();
}

/* read trips in new (CSV) format
 * note:
 * (1) order of columns is changes;
 * (2) instead of trip ID, we have driver ID, this is replaced
 * (3) timestamps are real ones, we replace them to limit to one day
 */
void wte_trips::read_trips_csv(read_table2& rt, const read_csv_filter& filt, bool clear_trips, bool chicago_format) {
	if(filt.speed_min < 0 || filt.speed_max < 0) throw std::runtime_error("wte_trips::read_trips_csv(): invalid parameters (speed cannot be negative)!\n");
	if(filt.dist_min > 0 || filt.dist_max > 0 || filt.speed_min > 0 || filt.speed_max > 0) {
		if(!spindex) throw std::runtime_error("wte_trips::read_trips_csv(): need to open distances matrix first if filtering based on distance or speed is required!\n");
	}
	
	std::unordered_set<unsigned int> nodes_filter;
	if(filt.nodes_file) {
		read_table2 rtn(filt.nodes_file);
		while(rtn.read_line()) {
			unsigned int id;
			if(!rtn.read(id)) break;
			nodes_filter.insert(id);
		}
		if(rtn.get_last_error() != T_EOF) {
			std::string ex_str = rtn.exception_string("wte_trips::read_trips_csv(): Error reading trips:\n");
			throw std::runtime_error(ex_str);
		}
	}
	std::uniform_int_distribution<unsigned int> start_extra_dist(0, 899);
	
	unsigned int tmin = filt.tmin;
	unsigned int tmax = filt.tmax;
	
	if(clear_trips) {
		trips.clear();
		i = 0;
		sorted = true;
	}
	rt.set_delim(',');
	size_t lines = 0;
	while(rt.read_line()) {
		lines++;
		trip t;
		unsigned int travel_time;
		{
			bool rr;
			if(chicago_format) rr = rt.read(t.start_ts, t.end_ts, t.start_node, t.end_node, travel_time);
			else rr = rt.read(read_table_skip(), t.start_ts, t.end_ts, t.start_node, t.end_node);
			if(!rr) break;
		}
		t.id = trips.size();
		
		/* filter based on start and end node */
		if(filt.nodes_file && !(nodes_filter.count(t.start_node) && nodes_filter.count(t.end_node))) continue;
		if(t.start_node == t.end_node) continue;
		
		/* filter based on start time */
		if(tmin > 0 || tmax > 0) {
			unsigned int t1 = t.start_ts;
			if(filt.tlimit_in_day) t1 = t1 % 86400;
			if(tmin > 0 && t1 < tmin) continue;
			if(tmax > 0 && t1 > tmax) continue;
		}
		
		/* if no end time is given, calculate it from the travel time matrix */
		if(!t.end_ts) {
			if(!ttindex) throw std::runtime_error("Need to load travel time matrix first if trips don't have end times!\n");
			ttime_t tt;
			uint8_t hour = (uint8_t)( (t.start_ts % 86400) / 3600);
			if(!ttindex->get_travel_time_id(t.start_node, t.end_node, hour, tt))
				throw std::runtime_error("Error calculating travel time!");
			t.end_ts = t.start_ts + tt;
		}
		
		/* filter based on trip duration, distance or speed */
		if(t.end_ts < t.start_ts) continue; // skip bad trips
		if(t.end_ts - t.start_ts <= filt.length_min) continue;
		if(filt.length_max > 0 && t.end_ts - t.start_ts > filt.length_max) continue;
		if(filt.dist_min > 0 || filt.dist_max > 0 || filt.speed_min > 0 || filt.speed_max > 0) {
			unsigned int d1 = spindex->get_dist_ix(t.start_node, t.end_node);
			if(d1 < filt.dist_min || (filt.dist_max > 0 && d1 > filt.dist_max)) continue;
			if(filt.speed_min > 0 || filt.speed_max > 0) {
				double speed = ((double)d1) / ((double)(t.end_ts - t.start_ts));
				if(filt.speed_min > 0 && speed < filt.speed_min) continue;
				if(filt.speed_max > 0 && speed > filt.speed_max) continue;
			}
		}
		
		/* Chicago: adjust start time, use travel time read separately */
		if(chicago_format) {
			t.start_ts += start_extra_dist(rng);
			t.end_ts = t.start_ts + travel_time;
		}
		
		trips.push_back(t);
		check_trips_last_sorted();
	}
	if(rt.get_last_error() != T_EOF || !lines) {
		std::string ex_str = rt.exception_string("wte_trips::read_trips_csv(): Error reading trips:\n");
		throw std::runtime_error(ex_str);
	}
	if(!sorted) sort_trips();
}

void wte_trips::read_trips_csv(const char* fn, const read_csv_filter& filt, const char* compress, bool clear_trips, bool chicago_format) {
	FILE* f = nullptr;
	if(compress && *compress) {
		size_t len = 20 + strlen(fn);
		char* cmd = (char*)malloc(sizeof(char)*len);
		if(!cmd) throw std::runtime_error("wte_trips::read_trips_csv(): Cannot allocate memory!\n");
		if(!strcmp(compress, "gz") || !strcmp(compress, "gzip"))
			snprintf(cmd, len, "/bin/gzip -cd %s", fn);
		else if (!strcmp(compress, "bz") || !strcmp(compress, "bz2") || !strcmp(compress, "bzip2"))
			snprintf(cmd, len, "/bin/bzip2 -cd %s", fn);
		else if (!strcmp(compress, "xz") || !strcmp(compress, "lzma"))
			snprintf(cmd, len, "/usr/bin/xz -cd %s", fn);
		else {
			free(cmd);
			throw std::runtime_error("wte_trips::read_trips_csv(): Unknown compression method given!\n");
		}
		f = popen(cmd, "re");
		free(cmd);
	}
	else f = fopen(fn, "r");
	if(!f) throw std::runtime_error("wte_trips::read_trips_csv(): Cannot open input file!\n");
	read_table2 rt(f);
	rt.set_fn(fn);
	read_trips_csv(rt, filt, clear_trips, chicago_format);
	if(compress) pclose(f);
	else fclose(f);
}


/* add initial set of drivers */
void wte_trips::add_drivers(size_t n, double tavg, bool use_end_locations) {
	if(trips.empty()) throw std::runtime_error("wte_trips::add_drivers(): This function needs trips to be loaded first!\n");
	double t1 = 0.0;
	double n1 = 0.0;
	if(tavg >= 0.0) t1 = tavg;
	else {
		for(const auto& x : trips) { t1 += (x.end_ts - x.start_ts); n1++; }
		if(n1 > 0.0) t1 /= n1; /* average trip time */
	}
	unsigned int t2 = (unsigned int)round(t1);
	std::uniform_int_distribution<unsigned int> time_dist(0,t2);
	std::uniform_int_distribution<size_t> node_dist(0,trips.size() - 1);
	tp = trips[0].start_ts;
	
	for(size_t i=0;i<n;i++) {
		const trip& t = trips[node_dist(rng)];
		unsigned int ptid = use_end_locations ? t.end_node : t.start_node;
		unsigned int t3 = time_dist(rng);
		add_driver(ptid, tp + t3);
	}
}

bool wte_trips::process_next_trip(unsigned int tmax1) {
	if(pr < 1.0) for(;i<trips.size();i++) if(prdist(rng)<pr) break;
	
	if(is_end()) return false;
	if(!tmax1) tmax1 = pwait(rng);
	bool res = process_trip(trips[i],tmax1);
	i++;
	return res;
}

void wte_trips::sort_trips(bool reassign_ids) {
	std::sort(trips.begin(),trips.end(),sort_tips_by_time());
	if(reassign_ids) for(size_t i = 0; i < trips.size(); i++)
		trips[i].id = i;
	sorted = true;
}

void wte_trips::copy_filtered_trips(const std::vector<trip>& trips1, const std::unordered_set<uint64_t>& trip_filter) {
	trips.clear();
	std::copy_if(trips1.begin(), trips1.end(), std::back_inserter(trips), [&trip_filter](const trip& t) -> bool { return trip_filter.count(t.id); });
}

void wte_trips::filter_trips(const std::unordered_set<uint64_t>& trip_filter) {
	auto it = std::remove_if(trips.begin(), trips.end(), [&trip_filter](const trip& t) -> bool { return trip_filter.count(t.id); });
	trips.erase(it, trips.end());
}



void wte::prune_drivers() {
	if(!driver_max_idle_time) return;
	safe_iter(moving_drivers, [this](const auto& x) {
		driver_id id = x.first;
		const auto& s = drivers_summary.at(id);
		if(s.end_time > s.start_time && tp > s.end_time && tp - s.end_time > driver_max_idle_time) remove_driver(id);
		return true;
	});
	safe_iter(drivers_nodes, [this](const auto& x) {
		driver_id id = x.first;
		const auto& s = drivers_summary.at(id);
		if(s.end_time > s.start_time && tp > s.end_time && tp - s.end_time > driver_max_idle_time) remove_driver(id);
		return true;
	});
}

