/*
 * ttestimate.cpp -- Travel time estimation algorithm from the paper:
 * Santi, P., Resta, G., Szell, M., Sobolevsky, S., Strogatz, S. H., & Ratti, C. (2014).
 * Quantifying the benefits of vehicle pooling with shareability networks. PNAS, 111(37), 13290--13294.
 * https://doi.org/10.1073/pnas.1403657111
 * 
 * Copyright 2020 Daniel Kondor <kondor.dani@gmail.com>
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
 */


#include <stdio.h>
#include <stdlib.h>

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <algorithm>
#include <cmath>

#include "read_table.h"

struct edge_info {
	explicit edge_info(double dist_) : dist(dist_), tt(0.0), tt2(0.0) { }
	edge_info(): dist(0.0), tt(0.0), tt2(0.0) { }
	double dist; /* in meters */
	double tt; /* in seconds */
	double tt2; /* updated travel time, in seconds */
	double offset; /* weighted difference in trip time */
	bool seen;
	/* ... */
};

struct node_pair_data {
	std::vector<unsigned int> trip_travel_times; /* used only in the beginning */
	std::vector<unsigned int> path;
	double total_length; /* updated by search_path() in each iteration */
	double avg_travel_time; /* calculated in the beginning */
	unsigned int cnt_trips;
	/* ... */
};

typedef std::unordered_map<unsigned int, std::unordered_map<unsigned int, edge_info> > network_type;

template<class dist_extractor>
bool search_path(const network_type& n, unsigned int start, std::unordered_map<unsigned int, node_pair_data>& targets, const dist_extractor& de) {
	
	struct node {
		double d; /* current estimate of distance to this node */
		unsigned int node_id; /* node id */
		bool operator < (const node& n) const {
			/* note: node_id is part of the comparison so that nodes can be found exactly */
			return d < n.d || (d == n.d && node_id < n.node_id);
		}
	};
	
	std::set<node> q; /* queue of nodes to process by distance */
	std::unordered_map<unsigned int, double> node_distances; /* distance of all nodes from the start node */
	node_distances[start] = 0;
	q.insert(node {0.0,start});
	std::unordered_map<unsigned int, unsigned int> ancestor_map;
	ancestor_map[start] = start;
	size_t found = 0;
	
	do {
		auto it = q.begin();
		unsigned int current = it->node_id;
		double d = it->d;
		q.erase(it);
		
		auto it2 = targets.find(current);
		if(it2 != targets.end()) {
			found++;
			it2->second.total_length = d;
		}
		/* exit if found all points */
		if(found == targets.size()) break;
		/* add to the queue the nodes reachable from the current */
		for(const auto& x : n.at(current)) {
			unsigned int n1 = x.first; /* node ID */
			double d1 = d + de(x.second); /* total distance this way */
			
			auto res = node_distances.insert(std::make_pair(n1, d1));
			if(res.second) {
				/* this node was not seen yet, we can add to the queue */
				q.insert(node{d1,n1});
				ancestor_map[n1] = current;
			}
			else {
				/* node already found, may need to be updated -- only if new distance is shorter */
				auto it3 = res.first;
				if(d1 < it3->second) {
					if(q.erase(node{it3->second,n1}) != 1) {
						fprintf(stderr, "search_path(): Error: node %u not found in the queue (distance: %f)!\n", n1, it3->second);
						return false;
					}
					it3->second = d1;
					q.insert(node{d1,n1});
					ancestor_map[n1] = current;
				}
			}
		}
	} while(q.size());
	
	if(found != targets.size()) {
		fprintf(stderr, "search_path(): Error: not all targets found!\n");
		return false;
	}
	
	/* find the path for all cases */
	for(auto& x : targets) {
		node_pair_data& np = x.second;
		np.path.clear();
		unsigned int current = x.first;
		size_t steps = 0;
		do {
			if(steps > ancestor_map.size()) {
				fprintf(stderr, "search_path(): Could not reconstruct path!\n");
				return false;
			}
			np.path.push_back(current);
			current = ancestor_map.at(current);
			steps++;
		} while(current != start);
		np.path.push_back(start);
		std::reverse(np.path.begin(), np.path.end());
	}
	
	return true;
}


int main(int argc, char **argv)
{
	const char* tripsfn = 0; /* set of trips to use */
	const char* networkfn = 0; /* street network with distances */
	char* node_filter_fn = 0; /* only include trips and the network from this set */
	
	const char* avg_times_out = 0; /* output average travel times between each pair here */
	
	unsigned int min_trip_time = 120; /* minimum time for trips (in seconds) */
	unsigned int max_trip_time = 7200; /* maximum time for trips */
	double min_trip_speed = 0.5; /* minimum trip speed (in m/s) */
	double max_trip_speed = 30; /* maximum trip speed (in m/s) */
	
	bool use_weights = false;
	bool weight_errors = false;
	
	unsigned int min_trip_start_hour = 0; /* minimum start hour for trips */
	unsigned int max_trip_start_hour = 0; /* maximum start hour for trips */
	
	unsigned int dow_min = 0; /* minimum day of week (filter days outside) */
	unsigned int dow_max = 7; /* maximum day of week (exclusive) */
	
	bool have_trip_id = true; /* whether the input file includes trip IDs (or taxi IDs) in the first column -- this is ignored */
	bool have_travel_time = false; /* whether the input file includes a separate last column with the real travel times or travel times should be calculated from the start and end times */
	bool ignore_errors = false; /* ignore lines with errors in the input */
	
	size_t max_it = 0;
	
	for(int i = 1; i < argc; i++) {
		if(argv[i][0] == '-') switch(argv[i][1]) {
			case 't':
				tripsfn = argv[i+1];
				i++;
				break;
			case 'n':
				networkfn = argv[i+1];
				i++;
				break;
			case 'T':
				min_trip_time = atoi(argv[i+1]);
				max_trip_time = atoi(argv[i+2]);
				i += 2;
				break;
			case 'v':
				min_trip_speed = atof(argv[i+1]);
				max_trip_speed = atof(argv[i+2]);
				i += 2;
				break;
			case 'h':
				min_trip_start_hour = atoi(argv[i+1]) * 3600;
				max_trip_start_hour = atoi(argv[i+2]) * 3600;
				i += 2;
				break;
			case 'a':
				avg_times_out = argv[i+1];
				i++;
				break;
			case 'f':
				node_filter_fn = argv[i+1];
				i++;
				break;
			case 'w':
				use_weights = true;
				break;
			case 'W':
				weight_errors = true;
				break;
			case 'd':
				/* day-of-week filter */
				if(i + 1 < argc && argv[i+1][0] != '-') {
					dow_min = atoi(argv[i+1]);
					i++;
					if(dow_min >= 7) {
						fprintf(stderr, "Invalid parameter for day-of-week filter: %s!\n", argv[i]);
						dow_min = 0;
					}
					else {
						if(i + 1 < argc && argv[i+1][0] != '-') {
							dow_max = atoi(argv[i+1]);
							i++;
							if(dow_max > 7 || dow_max <= dow_min) {
								fprintf(stderr, "Invalid parameters for day-of-week filter: %s %s!\n", argv[i-1], argv[i]);
								dow_max = 7;
							}
						}
						else dow_max = dow_min + 1;
					}
				}
				else fprintf(stderr, "Invalid parameter for day-of-week filter!\n");
				break;
			case 'I':
				have_trip_id = false;
				break;
			case 'S':
				have_travel_time = true;
				break;
			case 'E':
				ignore_errors = true;
				break;
			case 'm':
				max_it = atoi(argv[i+1]);
				i++;
				break;
			default:
				fprintf(stderr, "Unknown parameter: %s!\n", argv[i]);
				break;
		}
		else fprintf(stderr, "Unknown parameter: %s!\n", argv[i]);
	}
	
	if(! (tripsfn || networkfn) ) {
		fprintf(stderr, "At least one of the trips and network input file need to be specified!\n");
		return 1;
	}
	
	std::unordered_map<unsigned int, std::unordered_map<unsigned int, node_pair_data> > pairs;
	network_type n;
	{
		std::unordered_set<unsigned int> node_filter;
		if(node_filter_fn) {
			read_table2 rt(node_filter_fn);
			while(rt.read_line()) {
				unsigned int nn;
				if(!rt.read(nn)) break;
				node_filter.insert(nn);
			}
			if(rt.get_last_error() != T_EOF) {
				fprintf(stderr, "Error reading the node filter:\n");
				rt.write_error(stderr);
				return 1;
			}
		}
		
		/* read all trips */
		{
			read_table2 rt(tripsfn, stdin);
			size_t ntrips = 0;
			rt.set_delim(',');
			bool read_error = false;
			while(rt.read_line()) {
				unsigned int start_node, end_node, travel_time;
				unsigned int start_time, end_time;
				
				if(have_trip_id) {
					string_view_custom trip_id;
					if(!rt.read(trip_id)) read_error = true; /* we don't care about trip / taxi ID in the data */
				}
				
				if(!read_error) if(!rt.read(start_time, end_time, start_node, end_node)) read_error = true;
				if(start_node == end_node) continue;
				if(have_travel_time) {
					if(!read_error) if(!rt.read(travel_time)) read_error = true;
				}
				else {
					if(end_time <= start_time) continue;
					travel_time = end_time - start_time;
				}
				
				if(read_error) {
					if(ignore_errors) {
						read_error = false;
						continue;
					}
					else break;
				}
				
				if(node_filter_fn && !(node_filter.count(start_node) && node_filter.count(end_node))) continue;
				
				unsigned int start_hour = start_time % 86400;
				if(start_hour < min_trip_start_hour) continue;
				if(max_trip_start_hour && start_hour >= max_trip_start_hour) continue;
				if(travel_time < min_trip_time) continue;
				if(max_trip_time && travel_time > max_trip_time) continue;
				
				/* day-of-week filter */
				if(dow_min > 0 || dow_max < 7) {
					unsigned int dow = (start_time / 86400 + 3) % 7;
					if(dow < dow_min || dow >= dow_max) continue;
				}
				
				pairs[start_node][end_node].trip_travel_times.push_back(travel_time);
				ntrips++;
			}
			if(read_error || rt.get_last_error() != T_EOF) {
				fprintf(stderr, "Error reading trips:\n");
				rt.write_error(stderr);
				fprintf(stderr, "%s\n", rt.get_line_str());
				return 1;
			}
			fprintf(stderr, "%lu trips read\n", ntrips);
		}
		
		/* read the network */
		{
			read_table2 rt(networkfn, stdin);
			rt.set_delim(',');
			while(rt.read_line()) {
				unsigned int n1, n2;
				double d1;
				if(!rt.read(n1, n2, d1)) break;
				
				if(node_filter_fn && !(node_filter.count(n1) && node_filter.count(n2))) continue;
				
				n[n1][n2] = edge_info(d1);
			}
			if(rt.get_last_error() != T_EOF) {
				fprintf(stderr, "Error reading the network:\n");
				rt.write_error(stderr);
				return 1;
			}
		}
	}
	
	/* calculate distances and paths for all trips */
	for(auto& x : pairs)
		if(!search_path(n, x.first, x.second, [](const edge_info& e) { return e.dist; })) return 1;
	
	/* calculate travel speed for each trip, filter out too slow or too fast trips, calculate average travel times */
	{
		FILE* fout = 0;
		if(avg_times_out) {
			fout = fopen(avg_times_out, "w");
			if(!fout) {
				fprintf(stderr, "Cannot open output file %s!\n", avg_times_out);
				return 1;
			}
		}
		size_t filtered = 0, remaining = 0;
		std::vector<std::pair<unsigned int, unsigned int> > pairs_to_remove;
		double total_dist = 0.0;
		double total_time2 = 0.0;
		for(auto& y : pairs) for(auto& x : y.second) {
			double dist = x.second.total_length; /* travel distance in meters */
			double total_time = 0.0;
			unsigned int cnt = 0;
			for(unsigned int tt : x.second.trip_travel_times) {
				double trip_speed = dist / tt;
				if(min_trip_speed > 0.0 && trip_speed < min_trip_speed) {
					filtered++;
					continue;
				}
				if(max_trip_speed > 0.0 && trip_speed > max_trip_speed) {
					filtered++;
					continue;
				}
				total_time += tt;
				cnt++;
			}
			if(!cnt) {
				pairs_to_remove.push_back(std::make_pair(y.first, x.first));
				continue;
			}
			x.second.avg_travel_time = total_time / cnt;
			x.second.cnt_trips = cnt;
			total_time2 += total_time;
			total_dist += cnt * dist;
			remaining += cnt;
			if(fout) fprintf(fout, "%u, %u, %u, %f, %f\n", y.first, x.first, cnt, dist, x.second.avg_travel_time);
			
			x.second.trip_travel_times.clear(); /* no need to keep individual times anymore */
		}
		if(fout) fclose(fout);
		fprintf(stderr, "Filtered out %lu trips, remaining %lu\n", filtered, remaining);
		
		for(auto x : pairs_to_remove) {
			pairs[x.first].erase(x.second);
			if(pairs[x.first].empty()) pairs.erase(x.first);
		}
		
		size_t total_pairs = 0;
		for(const auto& x : pairs) total_pairs += x.second.size();
		double avg_speed = total_dist / total_time2;
		
		fprintf(stderr, "Total pairs: %lu, avg. speed: %f m/s (%f km/h)\n", total_pairs, avg_speed, avg_speed * 3.6);
		
		/* update initial travel time for all edges, based on the average travel speed */
		for(auto& x : n) for(auto& y : x.second) y.second.tt = y.second.dist / avg_speed;
	}
	
	size_t it = 0;
	while(!max_it || it < max_it) {
		it++;
		/* 1. re-calculate paths, based on the current assumed travel speeds */
		for(auto& x : pairs)
			if(!search_path(n, x.first, x.second, [](const edge_info& e) { return e.tt; })) return 1;
		/* total_length is the calculated travel time, path includes the travel path */
		double rel_err = 0.0;
		for(const auto& x : pairs) for(const auto& y : x.second) {
			double rel_err1 = (y.second.total_length - y.second.avg_travel_time) / y.second.avg_travel_time;
			if(weight_errors) rel_err1 *= y.second.cnt_trips;
			rel_err += std::abs(rel_err1);
		}
		
		/* 2. re-calculate offsets */
		for(auto& x : n) for(auto& y : x.second) {
			y.second.offset = 0.0;
			y.second.seen = false;
		}
		for(const auto& x : pairs) for(const auto& y : x.second) {
			double original_travel_time = y.second.avg_travel_time;
			double current_travel_time = y.second.total_length;
			for(size_t i = 0; i + 1 < y.second.path.size(); i++) {
				unsigned int n1 = y.second.path[i];
				unsigned int n2 = y.second.path[i+1];
				edge_info& ei = n.at(n1).at(n2);
				double factor = use_weights ? ei.tt / current_travel_time : 1.0;
				ei.offset += (current_travel_time - original_travel_time) * factor * y.second.cnt_trips;
				ei.seen = true;
			}
		}
		
		/* 3. inner loop -- adjust travel times based on offsets */
		bool again = false;
		double rel_err_new = 0.0;
		double k;
		for(k = 1.2; k >= 1.0001; k = 1 + (k - 1) * 0.75) {
			/* 3.1. new weights */
			for(auto& x : n) for(auto& y : x.second) {
				if(y.second.offset < 0.0) y.second.tt2 = y.second.tt * k;
				else if(y.second.offset > 0.0) y.second.tt2 = y.second.tt / k;
				else y.second.tt2 = y.second.tt;
			}
			
			/* 3.2. re-calculate relative errors */
			rel_err_new = 0.0;
			for(const auto& x : pairs) for(const auto& y : x.second) {
				double new_travel_time = 0.0;
				for(size_t i = 0; i + 1 < y.second.path.size(); i++) {
					unsigned int n1 = y.second.path[i];
					unsigned int n2 = y.second.path[i+1];
					const edge_info& ei = n.at(n1).at(n2);
					new_travel_time += ei.tt2;
				}
				double rel_err1 = (new_travel_time - y.second.avg_travel_time) / y.second.avg_travel_time;
				if(weight_errors) rel_err1 *= y.second.cnt_trips;
				rel_err_new += std::abs(rel_err1);
			}
			
			/* 3.3 decide whether to continue */
			if(rel_err_new < rel_err) {
				again = true;
				/* update the travel times on the network edges */
				for(auto& x : n) for(auto& y : x.second) y.second.tt = y.second.tt2;
				break;
			}
		}
		
		fprintf(stderr, "it: %lu, rel_err: %g, rel_err_new: %g, k: %f, again: %s\n", it, rel_err, rel_err_new, k, again ? "true" : "false");
		if(!again) break;
	}
	
	/* write out result -- note: we only write out edges that were included in the last iteration, i.e. at least one trip passses on them */
	FILE* out1 = stdout;
	for(const auto& x : n) for(const auto& y : x.second) {
		const edge_info& ei = y.second;
		if(ei.seen) fprintf(out1, "%u, %u, %f\n", x.first, y.first, ei.tt);
	}
	
	return 0;
}

