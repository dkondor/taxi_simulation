/*
 * create_path_matrix.cpp -- create ancestor map matrices based on
 * a travel time matrix
 * 
 * Note: this does not necessarily use the original network, since that
 * might not be available; instead, it allows using the already computed
 * travel time matrix, as if the network was fully connected. This is
 * based on the assumption that the shortest paths will typically
 * include the real network links. For better results, using the
 * original network is still recommended.
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


#include <stdio.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <set>
#include <algorithm>
#include <utility>
#include <array>
#include <thread>
#include <atomic>

#define USE_MUTEX
#include "mmap.h"
#include "ttimes2.h"


/* wrapper around a travel time index as a "network" */
template<class index_t>
struct tt_matrix_wrapper {
	protected:
		const tt_sorted_index<index_t>& ttindex;
		const index_t current_node = 0;
		const uint8_t hour = 0;
	public:
		explicit tt_matrix_wrapper(const tt_sorted_index<index_t>& ttindex_, index_t n, uint8_t h) : ttindex(ttindex_), current_node(n), hour(h) { }
		tt_matrix_wrapper at(index_t n) const { return tt_matrix_wrapper(ttindex, n, hour); }
		size_t count(index_t n) const { return ttindex.get_matrix_size(); }
		size_t size() const { return ttindex.get_matrix_size(); }
		uint8_t get_hour() const { return hour; }
		
		struct iterator {
			protected:
				const tt_index& ttindex;
				const index_t start = 0;
				const uint8_t hour = 0;
				const index_t* nodes = nullptr;
			
			public:
				typedef std::pair<index_t, uint16_t> value_type;
				typedef const iterator& reference;
				typedef std::input_iterator_tag iterator_category;
				
				reference operator * () const { return *this; }
				operator std::pair<index_t, uint16_t> () const { return std::make_pair(*nodes, ttindex.get_travel_time_id(start, *nodes, hour)); }
				iterator& operator++() { nodes++; return *this; }
				iterator operator++(int) { auto tmp = *this; nodes++; return tmp; }
				bool operator == (const iterator& it) const { return nodes == it.nodes; }
				bool operator != (const iterator& it) const { return nodes != it.nodes; }
				
				explicit iterator(const tt_sorted_index<index_t>& ttindex_, index_t start_, uint8_t h, size_t i = 0) :
					ttindex(ttindex_), start(start_), hour(h), nodes(ttindex_.get_sorted_ids_from(h, start_) + i) { }
		};
		
		iterator begin() const { return iterator(ttindex, current_node, hour, 0); }
		iterator end() const { return iterator(ttindex, current_node, hour, ttindex.get_matrix_size()); }
};

/* wrapper around the network, focusing on one hour */
template<class index_t>
struct network_wrapper {
	protected:
		const std::unordered_map<index_t, std::unordered_map<index_t, std::array<uint16_t, tt_index::hours> > >& n;
		const uint8_t hour = 0;
	public:
		explicit network_wrapper(const std::unordered_map<index_t, std::unordered_map<index_t, std::array<uint16_t, tt_index::hours> > >& n_, uint8_t h = 0) :
			n(n_), hour(h) { }
		
		struct node_wrapper {
			protected:
				const std::unordered_map<index_t, std::array<uint16_t, tt_index::hours> >& e;
				uint8_t hour = 0;
			public:
				explicit node_wrapper(const std::unordered_map<index_t, std::array<uint16_t, tt_index::hours> >& e_, uint8_t h) : e(e_), hour(h) { }
				struct iterator {
					protected:
						typename std::unordered_map<index_t, std::array<uint16_t, tt_index::hours> >::const_iterator it;
						uint8_t hour = 0;
					public:
						explicit iterator(typename std::unordered_map<index_t, std::array<uint16_t, tt_index::hours> >::const_iterator it_, uint8_t h) : it(it_), hour(h) { }
						typedef std::pair<index_t, uint16_t> value_type;
						typedef const iterator& reference;
						typedef std::input_iterator_tag iterator_category;
						
						reference operator * () const { return *this; }
						operator std::pair<index_t, uint16_t> () const { return std::make_pair(it->first, it->second[hour]); }
						iterator& operator++() { ++it; return *this; }
						iterator operator++(int) { return it++; }
						bool operator == (const iterator& it2) const { return it == it2.it; }
						bool operator != (const iterator& it2) const { return it != it2.it; }
				};
				
				iterator begin() const { return iterator(e.begin(), hour); }
				iterator end() const { return iterator(e.end(), hour); }
		};
		
		node_wrapper at(index_t node_id) const { return node_wrapper(n.at(node_id), hour); }
		size_t count(index_t node_id) const { return n.count(node_id); }
		size_t size() const { return n.size(); }
		uint8_t get_hour() const { return hour; }
};

template<class index_t>
struct node {
	unsigned int d; /* current estimate of distance to this node */
	index_t node_id; /* node id */
	bool operator < (const node& n) const {
		/* note: node_id is part of the comparison so that nodes can be found exactly */
		return d < n.d || (d == n.d && node_id < n.node_id);
	}
};

/* calculate ancestor map between all nodes from [0, matrix_size) */
template<class network_type, class index_t>
bool get_paths(const char* out_bin, index_t matrix_size, const network_type& n, std::atomic<index_t>& progress) {
	FileMappingT<index_t> out_matrix;
	if(!out_matrix.open_file(out_bin, FileMapping::Mode::ReadWrite, true)) {
		fprintf(stderr, "Error opening output file %s!\n",out_bin);
		return false;
	}
	
	size_t size2 = matrix_size;
	if(!out_matrix.change_file_size(size2 * size2 * sizeof(index_t))) {
		fprintf(stderr, "Error creating the output matrix!\n");
		return false;
	}
	if(!out_matrix.map_file(FileMapping::Mode::ReadWrite, true)) {
		fprintf(stderr, "Error creating the output matrix!\n");
		return false;
	}
	
	/* search for paths starting from every node */
	unsigned int searches = 0;
	for(index_t start_node = 0; start_node < matrix_size; start_node++) {
		if(!n.count(start_node)) continue; /* note: we leave out nodes that do not appear at all -- this is a hack for NYC, where node 0 is not used */
		uint64_t base = (uint64_t)start_node * size2;
		std::set<node<index_t> > q; /* queue of nodes to process by distance */
		std::unordered_map<index_t, unsigned int> node_distances; /* distance of all nodes from the start node */
		node_distances[start_node] = 0;
		out_matrix[base + start_node] = start_node;
		q.insert(node<index_t> {0, start_node});
		index_t found = 0;
		
		do {
			auto it = q.begin();
			index_t current = it->node_id;
			unsigned int d = it->d;
			q.erase(it);
			
			/* exit if found all points */
			found++;
			if(found == matrix_size) break;
			/* add to the queue the nodes reachable from the current */
			for(std::pair<index_t, uint16_t> x : n.at(current)) {
				index_t n1 = x.first; /* node ID */
				if(n1 == current) continue; /* avoid endless loop if the iteration also returns self-edges */
				unsigned int d1 = d; /* total weighted distance this way */
				d1 += x.second;
				auto it3 = node_distances.find(n1);
				if(it3 == node_distances.end()) { /* this node was not seen yet, we can add to the queue */
					node_distances.insert(std::make_pair(n1,d1));
					q.insert(node<index_t>{d1,n1});
					out_matrix[base + n1] = current;
				}
				else {
					/* node already found, may need to be updated -- only if new distance is shorter */
					if(d1 < it3->second) {
						if(q.erase(node<index_t>{it3->second,n1}) != 1) {
							fprintf(stderr,"Error: node %u not found in the queue (distance: %u)!\n",n1,it3->second);
							return false;
						}
						it3->second = d1;
						out_matrix[base + n1] = current;
						q.insert(node<index_t>{d1,n1});
					}
				}
			}
		} while(q.size());
		
		if(found != n.size()) {
			fprintf(stderr,"Not all points found!\n");
			return false;
		}
		searches++;
		if(searches > progress) {
			fprintf(stderr, "\r%u / %u start nodes processed", searches, matrix_size);
			fflush(stderr);
			progress = searches;
		}
	}
	out_matrix.close_file();
	return true;
}

struct params {
	const char* ttime_base = 0;
	const char* ttime_ext = ".bin";
	const char* index_base = 0;
	const char* index_ext = ".bin";
	const char* out_base = 0;
	const char* out_ext = ".bin";
	
	const char* network_fn = 0; /* if given, use the real network instead of doing the search based on the travel time matrix */
	
	unsigned int min_hour = 0;
	unsigned int max_hour = 24;
	
	unsigned int matrix_size = 0;
	
	int nthreads = 0;
	bool use_32bit = false;
	bool network_csv = false;
	bool only_max_time = false; /* do not do the processing, only calculate the maximum time along any edge */
	
	void parse_args(int argc, char** argv) {
		for(int i = 1; i < argc; i++) if(argv[i][0] == '-') switch(argv[i][1]) {
			case 't':
				ttime_base = argv[i+1];
				i++;
				break;
			case 'i':
				index_base = argv[i+1];
				i++;
				break;
			case 'o':
				out_base = argv[i+1];
				i++;
				break;
			case 'e':
				switch(argv[i][2]) {
					case 't':
						ttime_ext = argv[i+1];
						i++;
						break;
					case 'i':
						index_ext = argv[i+1];
						i++;
						break;
					case 'o':
						out_ext = argv[i+1];
						i++;
						break;
					case 0:
						ttime_ext = argv[i+1];
						index_ext = argv[i+1];
						out_ext = argv[i+1];
						i++;
						break;
					default:
						fprintf(stderr, "Unknown parameter: %s!\n", argv[i]);
						break;
				}
				break;
			case 'h':
				min_hour = atoi(argv[i+1]);
				max_hour = atoi(argv[i+2]);
				if(max_hour <= min_hour || min_hour > 23 || max_hour > 24)
					fprintf(stderr, "Invalid hour range: %s %s %s!\n", argv[i], argv[i+1], argv[i+2]);
				i += 2;
				break;
			case 'n':
				network_fn = argv[i+1];
				i++;
				break;
			case 'T':
				nthreads = atoi(argv[i+1]);
				i++;
				break;
			case '3':
				use_32bit = true;
				break;
			case 'c':
				network_csv = true;
				break;
			case 'm':
				matrix_size = atoi(argv[i+1]);
				i++;
				break;
			case 'M':
				only_max_time = true;
				break;
			default:
				fprintf(stderr, "Unknown parameter: %s!\n", argv[i]);
				break;
		}
		else fprintf(stderr, "Unknown parameter: %s!\n", argv[i]);
		
		if(nthreads == 0) nthreads = std::thread::hardware_concurrency();
		int nhours = max_hour - min_hour;
		if(nthreads > nhours) nthreads = nhours;
	}
};


template<class index_t>
void run_one_network(const std::unordered_map<index_t, std::unordered_map<index_t, std::array<uint16_t, tt_index::hours> > >& n,
		const char* out_base, const char* out_ext, uint8_t h, index_t matrix_size, std::atomic<index_t>& progress) {
	char* tmp = (char*)malloc(sizeof(char) * (strlen(out_base) + strlen(out_ext) + 4));
	if(!tmp) throw std::runtime_error("Cannot allocate memory!\n");
	sprintf(tmp, "%s%u%s", out_base, (unsigned int)h, out_ext);
	bool r = get_paths<network_wrapper<index_t>, index_t>(tmp, matrix_size, network_wrapper<index_t>(n, h), progress);
	free(tmp);
	if(!r) throw std::runtime_error("Error creating paths!\n");
}

template<class index_t>
void run_one_matrix(const tt_sorted_index<index_t>& ttindex, const char* out_base, const char* out_ext, uint8_t h, std::atomic<index_t>& progress) {
	char* tmp = (char*)malloc(sizeof(char) * (strlen(out_base) + strlen(out_ext) + 4));
	if(!tmp) throw std::runtime_error("Cannot allocate memory!\n");
	sprintf(tmp, "%s%u%s", out_base, (unsigned int)h, out_ext);
	bool r = get_paths<tt_matrix_wrapper<index_t>, index_t>(tmp, ttindex.get_matrix_size(), tt_matrix_wrapper<index_t>(ttindex, 0, h), progress);
	free(tmp);
	if(!r) throw std::runtime_error("Error creating paths!\n");
}

template<class index_t>
int run_all(const params& p) {
	if(!(p.index_base || p.network_fn)) {
		fprintf(stderr, "Missing input file names: either the travel time index or the road network needs to be given!\n");
		return 1;
	}
	tt_sorted_index<index_t> ttindex(p.matrix_size);
	ttindex.open_files(p.ttime_base, p.ttime_ext, true);
	if(p.index_base) ttindex.open_sorted_files(p.index_base, p.index_ext, true);
	
	std::atomic<index_t> progress(0);
	if(p.network_fn) {
		/* use a real network */
		std::unordered_map<index_t, std::unordered_map<index_t, std::array<uint16_t, tt_index::hours> > > n;
		uint16_t max_travel_time = 0;
		read_table2 rt(p.network_fn);
		if(p.network_csv) rt.set_delim(',');
		while(rt.read_line()) {
			index_t n1, n2;
			if(!rt.read(n1, n2)) break;
			for(uint8_t h = p.min_hour; h < p.max_hour; h++) {
				uint16_t tt1 = ttindex.get_travel_time_id(n1, n2, h);
				if(tt1 > max_travel_time) max_travel_time = tt1;
				n[n1][n2][h] = tt1;
			}
		}
		if(rt.get_last_error() != T_EOF) {
			fprintf(stderr, "Error reading network:\n");
			rt.write_error(stderr);
			return 1;
		}
		ttindex.close_files();
		if(p.only_max_time) {
			printf("%u\n", (unsigned int)max_travel_time);
			return 0;
		}
		
		if(p.nthreads > 1) {
			std::array<std::thread, tt_index::hours> threads;
			std::atomic<uint8_t> h(p.min_hour);
			for(int i = 0; i < p.nthreads; i++)
				threads[i] = std::thread([&n, &p, &h, &progress]() {
					while(true) {
						uint8_t h2 = h++;
						if(h2 >= p.max_hour) break;
						run_one_network<index_t>(n, p.out_base, p.out_ext, h2, p.matrix_size, progress);
					}
				});
			for(int i = 0; i < p.nthreads; i++) threads[i].join();
		}
		else for(uint8_t h = p.min_hour; h < p.max_hour; h++) run_one_network<index_t>(n, p.out_base, p.out_ext, h, p.matrix_size, progress);
	}
	else {
		if(p.only_max_time) {
			fprintf(stderr, "Calculating maximum times is only possible when using a real network!\n");
			return 1;
		}
		if(p.nthreads > 1) {
			std::array<std::thread, tt_index::hours> threads;
			std::atomic<uint8_t> h(p.min_hour);
			for(int i = 0; i < p.nthreads; i++)
				threads[i] = std::thread([&ttindex, &p, &h, &progress]() {
					while(true) {
						uint8_t h2 = h++;
						if(h2 >= p.max_hour) break;
						run_one_matrix<index_t>(ttindex, p.out_base, p.out_ext, h2, progress);
					}
				});
			for(int i = 0; i < p.nthreads; i++) threads[i].join();
		}
		else for(uint8_t h = p.min_hour; h < p.max_hour; h++) run_one_matrix<index_t>(ttindex, p.out_base, p.out_ext, h, progress);
	}
	
	return 0;
}

int main(int argc, char **argv)
{
	params p;
	p.parse_args(argc, argv);
	
	if(p.use_32bit) return run_all<uint32_t>(p);
	else return run_all<uint16_t>(p);
}

