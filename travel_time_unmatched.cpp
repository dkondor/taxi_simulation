/*
 * travel_time_unmatched.cpp -- calculate travel time on "remaining"
 * 	edges that were not assigned in the first phase
 * 
 * estimator algorithm from the following paper:
 * Santi, P., Resta, G., Szell, M., Sobolevsky, S., Strogatz, S. H., & Ratti, C. (2014).
 * Quantifying the benefits of vehicle pooling with shareability networks. PNAS, 111(37), 13290-13294.
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
	explicit edge_info(double dist_) : dist(dist_), tt(-1.0) { }
	edge_info(): dist(0.0), tt(-1.0) { }
	double dist; /* in meters */
	double tt; /* in seconds -- -1 means unknown */
	
};

struct unmatched_edge {
	unsigned int n1;
	unsigned int n2;
	unsigned int neighbors;
};

typedef std::unordered_map<unsigned int, std::unordered_map<unsigned int, edge_info> > network_type;


/* call the supplied function for each of the edge's neighbors */
template<class fn>
unsigned int edge_neighbors(const network_type& n, const std::unordered_map<unsigned int, std::unordered_set<unsigned int> >& reverse_edges,
		unsigned int n1, unsigned int n2, fn&& f) {
	unsigned int nn = 0;
	for(unsigned int j = 0; j < 2; j++) {
		unsigned int n3 = j ? n2 : n1;
		unsigned int n32 = j ? n1 : n2;
		auto it = n.find(n3);
		if(it != n.end()) for(const auto& y : it->second) if(y.first != n32) {
			f(y.second);
			nn++;
		}
		auto it2 = reverse_edges.find(n3);
		if(it2 != reverse_edges.end()) for(const auto& y : it2->second) if(y != n32) {
			const edge_info& ei = n.at(y).at(n3);
			f(ei);
			nn++;
		}
	}
	return nn;
}


int main(int argc, char **argv) {
	const char* networkfn = 0; /* street network with distances */
	const char* node_filter_fn = 0; /* only include trips and the network from this set */
	
	/* read estimated travel times from standard input, output new version to standard output */
	for(int i = 1; i < argc; i++) {
		if(argv[i][0] == '-') switch(argv[i][1]) {
			case 'n':
				networkfn = argv[i+1];
				i++;
				break;
			case 'f':
				node_filter_fn = argv[i+1];
				i++;
				break;
			default:
				fprintf(stderr, "Unknown parameter: %s!\n", argv[i]);
				break;
		}
		else fprintf(stderr, "Unknown parameter: %s!\n", argv[i]);
	}
	
	if(!networkfn) {
		fprintf(stderr, "Network input file need to be specified!\n");
		return 1;
	}
	
	network_type n;
	std::unordered_map<unsigned int, std::unordered_set<unsigned int> > reverse_edges;
	
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
	
	/* create opposite edges */
	for(const auto& x : n) for(const auto& y : x.second) {
		unsigned int n1 = x.first;
		unsigned int n2 = y.first;
		auto it = n.find(n2);
		if(it != n.end() && it->second.count(n1)) continue;
		reverse_edges[n2].insert(n1);
	}
	
	/* read the previous solution */
	{
		read_table2 rt(stdin);
		rt.set_delim(',');
		while(rt.read_line()) {
			unsigned int n1, n2;
			double tt;
			if(!rt.read(n1, n2, tt)) break;
			auto it = n.find(n1);
			if(it == n.end()) throw std::runtime_error("Edge not found!\n");
			auto it2 = it->second.find(n2);
			if(it2 == it->second.end()) throw std::runtime_error("Edge not found!\n");
			it2->second.tt = tt;
		}
	}
	
	/* for each unmatched edge, count the neighbors */
	while(true) {
		std::vector<unmatched_edge> u;
		unsigned int added = 0;
		for(const auto& x : n) for(const auto& y : x.second) {
			if(y.second.tt >= 0.0) continue; /* matched edge */
			/* count all the neighbors */
			unsigned int nn = 0;
			unsigned int n1 = x.first;
			unsigned int n2 = y.first;
			edge_neighbors(n, reverse_edges, n1, n2, [&nn](const edge_info& ei) {
				if(ei.tt > 0.0) nn++;
			});
			u.push_back({n1, n2, nn});
		}
		
		std::sort(u.begin(), u.end(), [](const unmatched_edge& x, const unmatched_edge& y) {
			return x.neighbors > y.neighbors;
		});
		
		for(const unmatched_edge& x : u) {
			/* average travel speed of neighbors, calculate travel time based on that */
			double sum_speed = 0.0;
			unsigned int n1 = x.n1;
			unsigned int n2 = x.n2;
			unsigned int nn = 0;
			edge_neighbors(n, reverse_edges, n1, n2, [&nn, &sum_speed](const edge_info& ei) {
				if(ei.tt > 0.0) {
					nn++;
					sum_speed += ei.dist / ei.tt;
				}
			});
			if(nn) {
				sum_speed /= (double)nn;
				edge_info& ei = n.at(n1).at(n2);
				ei.tt = ei.dist / sum_speed;
				added++;
			}
			else if(x.neighbors) throw std::runtime_error("No valid neighbors found!\n");
		}
		
		if(!added) break;
	}
	
	/* print out the result */
	FILE* fout = stdout;
	for(const auto& x : n) for(const auto& y : x.second)
		fprintf(fout, "%u, %u, %f\n", x.first, y.first, y.second.tt);
	
	return 0;
}

