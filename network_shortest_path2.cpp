/*
 * network_shortest_path.cpp -- calculate (shortest) distances between every pair of nodes in a weighted graph
 * 
 * output: stdout + optionally a binary matrix
 * 
 * Copyright 2019 Daniel Kondor <kondor.dani@gmail.com>
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
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <set>
#include <algorithm>
#include <utility>

#include "read_table.h"

#include "mmap.h"



struct node {
	double d; /* current estimate of distance to this node */
	uint64_t node_id; /* node id */
	bool operator < (const node& n) const {
		/* note: node_id is part of the comparison so that nodes can be found exactly */
		return d < n.d || (d == n.d && node_id < n.node_id);
	}
};


template<class dist_t>
int do_estimate(const char* out_bin, uint64_t matrix_size, const std::unordered_map<uint64_t, std::unordered_map<uint64_t,double > >& n) {
	FileMappingT<dist_t> out_matrix;
	if(!out_matrix.open_file(out_bin, FileMapping::Mode::ReadWrite, true)) {
		fprintf(stderr, "Error opening output file %s!\n",out_bin);
		return 1;
	}
	
	if(!out_matrix.change_file_size(matrix_size * matrix_size * sizeof(dist_t))) {
		fprintf(stderr, "Error creating the output matrix!\n");
		return 1;
	}
	if(!out_matrix.map_file(FileMapping::Mode::ReadWrite, true)) {
		fprintf(stderr, "Error creating the output matrix!\n");
		return 1;
	}
	
	size_t npoints = 0;
	for(const auto& x : n) if(x.first < matrix_size) npoints++;
	
	/* search for paths starting from every node */
	unsigned int searches = 0;
	for(const auto& x : n) {
		/* perform a search from each node that has assigned point */
		uint64_t start_node = x.first;
		if(start_node >= matrix_size) continue;
		
		std::set<node> q; /* queue of nodes to process by distance */
		std::unordered_map<uint64_t, double> node_distances; /* distance of all nodes from the start node */
		node_distances[start_node] = 0;
		q.insert(node {0.0,start_node});
		size_t found = 0;
		
		do {
			auto it = q.begin();
			uint64_t current = it->node_id;
			double d = it->d;
			q.erase(it);
			
			if(current < matrix_size) {
				double dtot = d;
				if(dtot > std::numeric_limits<dist_t>::max()) {
					fprintf(stderr, "Distance between nodes %lu -- %lu is too large (%f)!\n", start_node, current, dtot);
					return 1;
				}
				uint64_t off = start_node * matrix_size + current;
				out_matrix[off] = (dist_t)round(dtot);
				found++;
			}
			
			/* exit if found all points */
			if(found == npoints) break;
			
			/* add to the queue the nodes reachable from the current */
			for(const auto& x : n.at(current)) {
				uint64_t n1 = x.first; /* node ID */
				double d1 = d; /* total weighted distance this way */
				d1 += x.second;
				auto it3 = node_distances.find(n1);
				if(it3 == node_distances.end()) { /* this node was not seen yet, we can add to the queue */
					node_distances.insert(std::make_pair(n1, d1));
					q.insert(node{d1, n1});
				}
				else {
					/* node already found, may need to be updated -- only if new distance is shorter */
					if(d1 < it3->second) {
						if(q.erase(node{it3->second, n1}) != 1) {
							fprintf(stderr,"Error: node %lu not found in the queue (distance: %f)!\n", n1, it3->second);
							return 1;
						}
						it3->second = d1;
						q.insert(node{d1, n1});
					}
				}
			}
		} while(q.size());
		
		if(found != npoints) {
			fprintf(stderr,"Not all points found!\n");
			return 1;
		}
		searches++;
		fprintf(stderr, "\r%u / %lu start nodes processed", searches, npoints);
		fflush(stderr);
	}
	out_matrix.close_file();	
	
	return 0;
}


int main(int argc, char **argv)
{
	char* network_fn = 0; /* input: network file (with distances for each edge) */
	char* out_bin = 0; /* binary output */
	uint64_t matrix_size = 0; /* size of binary output matrix */
	bool is_csv = false; /* should be true if the input file is CSV */
	bool use_16bit = false; /* set to true if matrix should contain 16-bit results */
	
	for(int i=1;i<argc;i++) {
		if(argv[i][0] == '-') switch(argv[i][1]) {
			case 'n':
				network_fn = argv[i+1];
				i++;
				break;
			case 'c':
				is_csv = true;
				break;
			case 'o':
				out_bin = argv[i+1];
				i++;
				break;
			case 'm':
				matrix_size = atoi(argv[i+1]);
				i++;
				break;
			case '1':
				use_16bit = true;
				break;
			default:
				fprintf(stderr,"Unknown parameter: %s!\n",argv[i]);
				break;
		}
		else fprintf(stderr,"Unknown parameter: %s!\n",argv[i]);
	}
	
	/* read the network */
	// graph is simply an associative container of edges with distances and counts (of trips using the edge)
	uint64_t max_ptid = 0;
	std::unordered_map<uint64_t,std::unordered_map<uint64_t,double > > n;
	{
		read_table2 rt(network_fn,stdin);
		if(is_csv) rt.set_delim(',');
		while(rt.read_line()) {
			uint64_t n1,n2;
			double d;
			if(!rt.read(n1,n2,d)) break;
			if(n1 > max_ptid) max_ptid = n1;
			if(n2 > max_ptid) max_ptid = n2;
			n[n1][n2] = d;
			//~ n[n2][n1] = d; -- uncomment this to use symmetric network
		}
		if(rt.get_last_error() != T_EOF) {
			fprintf(stderr,"Error reading network:\n");
			rt.write_error(stderr);
			return 1;
		}
	}
	
	/* resize the output matrix */
	if(!matrix_size) matrix_size = max_ptid + 1;
	
	if(use_16bit) do_estimate<uint16_t>(out_bin, matrix_size, n);
	else do_estimate<uint32_t>(out_bin, matrix_size, n);
}

