/*  -*- C++ -*-
 * ttimes.h -- read and process travel time data
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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <limits>
#include <utility>
#include <algorithm>
#include <stdexcept>
#include "read_table.h"

#include <thread>
#include <atomic>
#include <string>
#include <sstream>
		
/* memory mapped files to read travel time data */
#include "mmap.h"

/* include a mutex for modifying file mappings */
#ifdef USE_MUTEX
#include <mutex>
#endif


class tt_index {
	public:
		static constexpr uint8_t hours = 24;
	
	protected:	
		const unsigned int matrix_size;
		FileMappingT<uint16_t> mappings[hours];
		
#ifdef USE_MUTEX
		std::recursive_mutex mutex1;
		void lock_mutex() { mutex1.lock(); }
		void unlock_mutex() { mutex1.unlock(); }
#else
		void lock_mutex() { }
		void unlock_mutex() { }
#endif
		
		/* map file i to memory */
		bool map_file_nolock(uint8_t i) {
			if(i >= hours) return false;
			if(!mappings[i].is_mapped()) {
				return mappings[i].map_file();
			}
			else return true;
		}
		void unmap_file_nolock(uint8_t hour) {
			if(hour < hours) mappings[hour].unmap_file();
		}
		
		/* get travel time between known nodes */
		uint16_t get_time(unsigned int start, unsigned int end, uint8_t hour) const {
			uint64_t offset = start*matrix_size + end;
			return mappings[hour][offset];
		}
		
		/* close all open files */
		void close_files_nolock() {
			for(uint8_t i=0;i<hours;i++) {
				mappings[i].close_file();
			}
		}
		
		
	public:
		/* constructor: take the matrix size as an argument */
		explicit tt_index(unsigned int matrix_size_) : matrix_size(matrix_size_) { }
		/* default matrix size */
		tt_index() : tt_index(11790U) { }
		
		~tt_index() { close_files(); }
		
		unsigned int get_matrix_size() const { return matrix_size; }
		
		/* open all files with travel times
		 * filenames have the formate: base%uext
		 * not thread safe, should be called from one thread which has
		 * exclusive access to this instance 
		 * throws exception on error */
		void open_files(const char* base, const char* ext, bool map = false) {
			unsigned int len = strlen(base) + strlen(ext) + 4;
			char* buf = (char*)malloc(sizeof(char)*len);
			if(!buf) throw std::runtime_error("tt_index::open_files(): error allocating memory!\n");
			
			/* make sure no files are open */
			lock_mutex();
			close_files_nolock();
			
			bool ret = true;
			size_t total_size = matrix_size;
			total_size = total_size * total_size;
			std::ostringstream strs; // error string
			
			for(uint8_t i=0;i<hours;i++) {
				sprintf(buf,"%s%u%s",base,(unsigned int)i,ext);
				ret = mappings[i].open_file(buf);
				if(!ret || (map && !map_file_nolock(i))) {
					strs << "tt_index::open_files(): error opening file " << buf << "!\n";
					ret = false;
					break;
				}
				
				/* check if file size is as expected */
				if(mappings[i].size() != total_size) {
					strs << "tt_index::open_files(): size mismatch for file " << buf <<
						": expected " << total_size << " elements, got " << mappings[i].size() << "!\n";
					ret = false;
					break;
				}
			}
			
			if(!ret) close_files_nolock();
			free(buf);
			unlock_mutex();
			
			if(!ret) throw std::runtime_error(strs.str());
		}
		bool map_file(uint8_t i) {
			if(mappings[i].is_mapped()) return true;
			lock_mutex();
			bool ret = map_file_nolock(i);
			unlock_mutex();
			return ret;
		}
		
		/* close all open files */
		void close_files() {
			lock_mutex();
			close_files_nolock();
			unlock_mutex();
		}
		
		
		/* Get travel time between two points using a query with the IDs
		 * Returns 0 on success, 1 on error */
		bool get_travel_time_id(unsigned int start_id, unsigned int end_id, uint8_t hour, uint16_t& ttime) {
			if(hour >= hours) {
				fprintf(stderr,"tt_index::get_travel_time_id(): invalid hour parameter (must be < %u)!\n",(unsigned int)hours);
				return false;
			}
			if(!map_file(hour)) {
				fprintf(stderr,"tt_index::get_travel_time_id(): cannot access file for hour %u!\n",(unsigned int)hour);
				return false;
			}
			if(start_id >= matrix_size || end_id >= matrix_size) {
				fprintf(stderr,"tt_index::get_travel_time_id(): too large IDs: %u %u!\n",start_id,end_id);
				return false;
			}
			
			ttime = get_time(start_id,end_id,hour);
			return true;
		}
		
		uint16_t get_travel_time_id(unsigned int start_id, unsigned int end_id, uint8_t hour) {
			uint16_t ttime;
			if(!get_travel_time_id(start_id, end_id, hour, ttime)) return UINT16_MAX;
			else return ttime;
		}
		
		
		/* same but with const this */
		bool get_travel_time_id(unsigned int start_id, unsigned int end_id, uint8_t hour, uint16_t& ttime) const {
			if(hour >= hours) {
				fprintf(stderr,"tt_index::get_travel_time_id(): invalid hour parameter (must be < %u)!\n",(unsigned int)hours);
				return false;
			}
			if(!mappings[hour].is_mapped()) {
				fprintf(stderr,"tt_index::get_travel_time_id(): cannot access file for hour %u!\n",(unsigned int)hour);
				return false;
			}
			if(start_id >= matrix_size || end_id >= matrix_size) {
				fprintf(stderr,"tt_index::get_travel_time_id(): too large IDs: %u %u!\n",start_id,end_id);
				return false;
			}
			
			ttime = get_time(start_id,end_id,hour);
			return true;
		}
		
		uint16_t get_travel_time_id(unsigned int start_id, unsigned int end_id, uint8_t hour) const {
			uint16_t ttime;
			if(!get_travel_time_id(start_id, end_id, hour, ttime)) return UINT16_MAX;
			else return ttime;
		}
		
		/* unmap files that are no longer needed
		 * (e.g. if events are processed sequentially)
		 * thread safe in a limited way:
		 * 	multiple threads can call this function simultenously,
		 * 	but the caller should ensure no other thread calls get_travel_time()
		 * 	or get_total_travel_time() with the same hour parameter simultaneously */
		void unmap_file(uint8_t hour) {
			lock_mutex();
			unmap_file_nolock(hour);
			unlock_mutex();
		}
		void unmap_files_before(uint8_t hour) {
			lock_mutex();
			for(uint8_t i=0; i<hour && i<hours; i++) unmap_file_nolock(i);
			unlock_mutex();
		}
		
		/* try to ensure that all data is read into main memory */
		void prefault() {
			for(uint8_t i = 0; i < hours; i++) mappings[i].prefault();
		}
};


/* sort IDs according to travel time to be able to search for points that are close */
template<class index_t>
class tt_sorted_index : public tt_index {
	protected:
		index_t* sorted_matrix_from[hours];
		index_t* sorted_matrix_to[hours];
		
		FileMappingT<index_t> sorted_mappings[hours];
		
		unsigned int matrix_sort_cnt;
		
		void free_hour_nolock2(uint8_t h) {
			if(h < hours) sorted_mappings[h].unmap_file();
			sorted_matrix_from[h] = nullptr;
			sorted_matrix_to[h] = nullptr;
		}
		
		void free_hour_nolock(uint8_t h) {
			free_hour_nolock2(h);
			unmap_file_nolock(h);
		}
		
		bool map_sorted_file_nolock(uint8_t h) {
			if(h >= hours) return false;
			if(!sorted_mappings[h].is_mapped()) {
				if(!sorted_mappings[h].map_file()) return false;
				size_t s1 = matrix_size;
				s1 = s1*s1;
				sorted_matrix_from[h] = sorted_mappings[h].data();
				sorted_matrix_to[h] = sorted_matrix_from[h] + s1;
			}
			return true;
		}
		
		/* close all open files */
		void close_sorted_files() {
			lock_mutex();
			for(uint8_t i=0;i<hours;i++) {
				sorted_mappings[i].close_file();
			}
			unlock_mutex();
		}
		
		void alloc_sorted(uint8_t h) {
			uint64_t size = matrix_size;
			sorted_mappings[h].open_anon();
			sorted_mappings[h].change_file_size(size * size * 2UL * sizeof(index_t));
			if(!sorted_mappings[h].map_file(FileMapping::Mode::ReadWrite, false)) {
				unlock_mutex();
				throw std::runtime_error("tt_sorted_index::sort_hour(): error allocating memory!\n");
			}
		}
		
		
		void sort_hour_from_maybe_lock(uint8_t h, bool need_lock) {
			if(h >= hours) throw std::runtime_error("tt_sorted_index::sort_hour_from(): invalid hour parameter!\n");
			if(sorted_matrix_from[h]) return;
			if(need_lock) lock_mutex();
			if(!sorted_matrix_from[h]) {
				if(!map_file_nolock(h)) {
					if(need_lock) unlock_mutex();
					throw std::runtime_error("tt_sorted_index::sort_hour_from(): cannot open requested travel time file!\n");
				}
				
				if(sorted_mappings[h].is_open()) {
					bool res = true;
					if(!sorted_mappings[h].is_mapped()) res = map_sorted_file_nolock(h);
					if(need_lock) unlock_mutex();
					if(!res) throw std::runtime_error("tt_sorted_index::sort_hour_from(): cannot open requested index file!\n");
					return;
				}
				
				if(!sorted_mappings[h].is_mapped()) alloc_sorted(h);
				sorted_matrix_from[h] = sorted_mappings[h].data();
				if(need_lock) unlock_mutex();
				
				size_t size = matrix_size;
				for(index_t i=0;i<matrix_size;i++) {
					for(index_t j=0;j<matrix_size;j++) sorted_matrix_from[h][i*size + j] = j;
					std::sort(sorted_matrix_from[h] + i*size, sorted_matrix_from[h] + (i+1)*size,
						[mappings = mappings,h,i,size](index_t x, index_t y) {
							uint16_t ttime1 = mappings[h][i*size + x];
							uint16_t ttime2 = mappings[h][i*size + y];
							/* ensure that "travel" between the same locations is always the shortest */
							if(ttime1 == 0 && i != x) ttime1 = 1;
							if(ttime2 == 0 && i != y) ttime2 = 1;
							return ttime1 < ttime2;
						});
				}
				matrix_sort_cnt++;
			}
			else if(need_lock) unlock_mutex();
		}
		
		void sort_hour_to_maybe_lock(uint8_t h, bool need_lock) {
			if(h >= hours) throw std::runtime_error("tt_sorted_index::sort_hour_to(): invalid hour parameter!\n");
			if(sorted_matrix_to[h]) return;
			if(need_lock) lock_mutex();
			if(!sorted_matrix_to[h]) {
				if(!map_file_nolock(h)) {
					if(need_lock) unlock_mutex();
					throw std::runtime_error("tt_sorted_index::sort_hour_to(): cannot open requested travel time file!\n");
				}

				if(sorted_mappings[h].is_open()) {
					bool res = true;
					if(!sorted_mappings[h].is_mapped()) res = map_sorted_file_nolock(h);					
					if(need_lock) unlock_mutex();
					if(!res) throw std::runtime_error("tt_sorted_index::sort_hour_from(): cannot open requested index file!\n");
					return;
				}
				
				if(!sorted_mappings[h].is_mapped()) alloc_sorted(h);
				
				size_t size = matrix_size;
				sorted_matrix_to[h] = sorted_mappings[h].data() + size*size;
				if(need_lock) unlock_mutex();
				
				for(index_t i=0;i<matrix_size;i++) {
					for(index_t j=0;j<matrix_size;j++) sorted_matrix_to[h][i*size + j] = j;
					std::sort(sorted_matrix_to[h] + i*size, sorted_matrix_to[h] + (i+1)*size,
						[mappings = mappings,h,i,size](index_t x, index_t y) {
							uint16_t ttime1 = mappings[h][x*size + i];
							uint16_t ttime2 = mappings[h][y*size + i];
							/* ensure that "travel" between the same locations is always the shortest */
							if(ttime1 == 0 && i != x) ttime1 = 1;
							if(ttime2 == 0 && i != y) ttime2 = 1;
							return ttime1 < ttime2;
						}
					);
				}
				matrix_sort_cnt++;
			}
			else if(need_lock) unlock_mutex();
		}
		
	public:
		tt_sorted_index() : tt_index(), matrix_sort_cnt(0) {
			if((unsigned long long)std::numeric_limits<index_t>::max() < (unsigned long long)matrix_size)
				throw std::runtime_error("Index datatype is too small for matrix size!\n");
			for(uint8_t h = 0; h < hours; h++) {
				sorted_matrix_from[h] = nullptr;
				sorted_matrix_to[h] = nullptr;
			}
		}
		explicit tt_sorted_index(unsigned int matrix_size_) : tt_index(matrix_size_), matrix_sort_cnt(0) {
			if((unsigned long long)std::numeric_limits<index_t>::max() < (unsigned long long)matrix_size)
				throw std::runtime_error("Index datatype is too small for matrix size!\n");
			for(uint8_t h = 0; h < hours; h++) {
				sorted_matrix_from[h] = nullptr;
				sorted_matrix_to[h] = nullptr;
			}
		}
		
		void sort_hour_from(uint8_t h) { sort_hour_from_maybe_lock(h, true); }
		
		void sort_hour_to(uint8_t h) { sort_hour_to_maybe_lock(h, true); }
		
		void open_sorted_files(const char* base, const char* ext, bool map = false) {
			close_sorted_files();
			
			unsigned int len = strlen(base) + strlen(ext) + 4;
			char* buf = (char*)malloc(sizeof(char)*len);
			if(!buf) throw std::runtime_error("tt_sorted_index::open_sorted_files(): error allocating memory!\n");
			
			uint64_t fs = matrix_size;
			fs = 2UL*fs*fs;
			
			bool ret = true;
			
			std::ostringstream strs; // error string
			lock_mutex();
			
			for(uint8_t i=0;i<hours;i++) {
				
				sprintf(buf,"%s%u%s",base,(unsigned int)i,ext);
				ret = sorted_mappings[i].open_file(buf);
				if(!ret || (map && !map_sorted_file_nolock(i))) {
					strs << "tt_sorted_index::open_sorted_files(): error opening file " << buf << "!\n";
					ret = false;
					break;
				}
				/* check if file size is as expected */
				if(sorted_mappings[i].size() != fs) {
					strs << "tt_sorted_index::open_sorted_files(): size mismatch for file " << buf <<
						": expected " << fs << " elements, got " << sorted_mappings[i].size() << "!\n";
					ret = false;
					break;
				}
			}
			
			free(buf);
			unlock_mutex();
			if(!ret) {
				close_sorted_files();
				throw std::runtime_error(strs.str());
			}
		}
		
		bool save_sorted_files(const char* base, const char* ext, uint8_t min_hour = 0, uint8_t max_hour = 0, int nthreads = -1) {
			if(max_hour == 0) max_hour = hours;
			if(min_hour >= max_hour || min_hour >= hours || max_hour > hours) {
				fprintf(stderr,"tt_sorted_index::open_sorted_files(): Invalid minimum / maximum hour given!\n");
				return false;
			}
			
			unsigned int len = strlen(base) + strlen(ext) + 4;
			char* buf = (char*)malloc(sizeof(char)*len);
			if(!buf) {
				fprintf(stderr,"tt_sorted_index::open_sorted_files(): error allocating memory!\n");
				return false;
			}
			
			lock_mutex();
			{
				int nhours = max_hour - min_hour;
				if(nthreads > nhours) nthreads = nhours;
			}
			if(nthreads > 1) {
				std::vector<std::thread> threads;
				std::atomic<uint8_t> h(min_hour);
				for(int i = 0; i < nthreads; i++) {
					threads.emplace_back([&h, max_hour, this] () {
						while(true) {
							uint8_t h2 = h++;
							if(h2 >= max_hour) break;
							sort_hour_from_maybe_lock(h2, false);
							sort_hour_to_maybe_lock(h2, false);
						}
					});
				}
				for(int i = 0; i < nthreads; i++) threads[i].join();
			}
			else for(uint8_t i=min_hour;i<max_hour;i++) {
				sort_hour_from_maybe_lock(i, false);
				sort_hour_to_maybe_lock(i, false);
			}
			
			bool ret = true;
			
			for(uint8_t i=min_hour;i<max_hour;i++) {
				sprintf(buf,"%s%u%s",base,(unsigned int)i,ext);
				FILE* f = fopen(buf,"w");
				if(!f) {
					fprintf(stderr,"tt_sorted_index::save_sorted_files(): error opening file %s!\n",buf);
					ret = false;
					break;
				}
				
				size_t s1 = matrix_size;
				s1 = s1*s1;
				size_t s2 = fwrite(sorted_matrix_from[i],sizeof(index_t),s1,f);
				if(s2 != s1) {
					fprintf(stderr,"tt_sorted_index::save_sorted_files(): error writing file %s!\n",buf);
					ret = false;
					break;
				}
				s2 = fwrite(sorted_matrix_to[i],sizeof(index_t),s1,f);
				if(s2 != s1) {
					fprintf(stderr,"tt_sorted_index::save_sorted_files(): error writing file %s!\n",buf);
					ret = false;
					break;
				}
				if(fclose(f)) {
					fprintf(stderr,"tt_sorted_index::save_sorted_files(): error writing file %s!\n",buf);
					ret = false;
					break;
				}
			}
			free(buf);
			unlock_mutex();
			return ret;
		}
		
		const index_t* get_sorted_ids_from(uint8_t h, unsigned int id) {
			if(h >= hours) return 0;
			sort_hour_from(h);
			return sorted_matrix_from[h] + id*(uint64_t)matrix_size;
		}
		
		const index_t* get_sorted_ids_to(uint8_t h, unsigned int id) {
			if(h >= hours) return 0;
			sort_hour_to(h);
			return sorted_matrix_to[h] + id*(uint64_t)matrix_size;
		}
		
		const index_t* get_sorted_ids_from(uint8_t h, unsigned int id) const {
			if(h >= hours) return 0;
			if(sorted_matrix_from[h] == 0) return 0;
			return sorted_matrix_from[h] + id*(uint64_t)matrix_size;
		}
		
		const index_t* get_sorted_ids_to(uint8_t h, unsigned int id) const {
			if(h >= hours) return 0;
			if(sorted_matrix_to[h] == 0) return 0;
			return sorted_matrix_to[h] + id*(uint64_t)matrix_size;
		}
		
		void free_hour(uint8_t h) {
			if(h < hours && (sorted_matrix_from[h] || sorted_matrix_to[h] || sorted_mappings[h].is_mapped() || mappings[h].is_mapped())) {
				lock_mutex();
				free_hour_nolock(h);
				unlock_mutex();
			}
		}
		
		void free_before(uint8_t h) {
			lock_mutex();
			for(uint8_t i=0; i<h && i<hours; i++) free_hour_nolock(i);
			unlock_mutex();
		}
		
		unsigned int get_sort_cnt() const {
			return matrix_sort_cnt;
		}
		
		/* try to ensure that all data is read into main memory */
		void prefault() {
			tt_index::prefault();
			for(uint8_t i = 0; i < hours; i++) sorted_mappings[i].prefault();
		}
		
		~tt_sorted_index() {
			close_sorted_files();
		}
};


/* version to calculate path as well -- tt_index_base should be either
 * tt_index or tt_sorted_index */
template<class index_t, class tt_index_base>
class tt_path_index : public tt_index_base {
	protected:
		FileMappingT<index_t> ancestor_maps[tt_index_base::hours];
		/* map file i to memory */
		bool map_ancestor_file_nolock(uint8_t i) {
			if(i >= tt_index_base::hours) return false;
			if(!ancestor_maps[i].is_mapped()) {
				return ancestor_maps[i].map_file();
			}
			else return true;
		}
		void unmap_ancestor_file_nolock(uint8_t hour) {
			if(hour < tt_index_base::hours) ancestor_maps[hour].unmap_file();
		}
		
		/* close all open files */
		void close_ancestor_files_nolock() {
			for(uint8_t i=0;i<tt_index_base::hours;i++) {
				unmap_ancestor_file_nolock(i);
				ancestor_maps[i].close_file();
			}
		}
		
		
	public:
		explicit tt_path_index(unsigned int matrix_size_ = 0) : tt_index_base(matrix_size_) {
			if((unsigned long long)std::numeric_limits<index_t>::max() < (unsigned long long)matrix_size_)
				throw std::runtime_error("Index datatype is too small for matrix size!\n");
		}
		
		/* open and map all files with ancestor maps for path search */
		void open_ancestor_files(const char* base, const char* ext) {
			/* make sure no files are open */
			close_ancestor_files();
			
			unsigned int len = strlen(base) + strlen(ext) + 4;
			char* buf = (char*)malloc(sizeof(char)*len);
			if(!buf) throw std::runtime_error("tt_path_index::open_ancestor_files(): error allocating memory!\n");
			
			std::ostringstream strs; // error string
			bool ret = true;
			
			size_t total_size = this->matrix_size;
			total_size = total_size * total_size;
			this->lock_mutex();
			
			for(uint8_t i=0;i<tt_index_base::hours;i++) {
				sprintf(buf,"%s%u%s",base,(unsigned int)i,ext);
				ret = ancestor_maps[i].open_file(buf) && map_ancestor_file_nolock(i);
				if(!ret) {
					strs << "tt_path_index::open_ancestor_files(): error opening file " << buf << "%s!\n";
					break;
				}
				
				/* check if file size is as expected */
				if(ancestor_maps[i].size() != total_size) {
					strs << "tt_path_index::open_ancestor_files(): size mismatch for file " << buf <<
						": expected " << total_size << " elements, got " << ancestor_maps[i].size() << "!\n";
					ret = false;
					break;
				}
			}
			
			free(buf);
			if(!ret) close_ancestor_files_nolock();
			this->unlock_mutex();
			if(!ret) throw std::runtime_error(strs.str());
		}
		/* close all open files */
		void close_ancestor_files() {
			this->lock_mutex();
			close_ancestor_files_nolock();
			this->unlock_mutex();
		}
		
		/* find the path from a source to a destination;
		 * if reverse == false, the path is returned in reversed order (can be useful if the caller wants to insert additional nodes at the beginning) */
		bool find_path(index_t start_id, index_t end_id, uint8_t hour, std::vector<index_t>& path, std::vector<uint16_t>& dist, bool reverse = true) const {
			if(start_id >= this->matrix_size || end_id >= this->matrix_size) {
				fprintf(stderr, "tt_path_index::find_path(): node IDs are too large!\n");
				return false;
			}
			if(hour >= tt_index_base::hours) {
				fprintf(stderr,"tt_path_index::find_path(): invalid hour parameter (must be < %u)!\n",(unsigned int)tt_index_base::hours);
				return false;
			}
			if(!(this->mappings[hour].is_mapped() && ancestor_maps[hour].is_mapped())) {
				fprintf(stderr,"tt_path_index::find_path(): cannot access travel time or path file for hour %u!\n",(unsigned int)hour);
				return false;
			}
			
			path.clear();
			dist.clear();
			index_t current = end_id;
			uint16_t d = 0;
			size_t base = ((size_t)this->matrix_size) * start_id;
			
			unsigned int steps = 0;
			while(true) {
				path.push_back(current);
				dist.push_back(d);
				if(current == start_id) break;
				if(steps > this->matrix_size) {
					fprintf(stderr, "tt_path_index::find_path(): start node not found after %u maximum steps!\n", steps);
					return false;
				}
				index_t next = ancestor_maps[hour][base + current];
				d = this->get_travel_time_id(next, current, hour);
				current = next;
				steps++;
			}
			
			if(reverse) {
				/* need to reverse path and dist */
				std::reverse(path.begin(), path.end());
				std::reverse(dist.begin(), dist.end());
			}
			return true;
		}
		
		/* try to ensure that all data is read into main memory */
		void prefault() {
			tt_index_base::prefault();
			for(uint8_t i = 0; i < tt_index_base::hours; i++) ancestor_maps[i].prefault();
		}
		
};

