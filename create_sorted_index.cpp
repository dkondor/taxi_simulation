/*
 * create_sorted_index.cpp -- create a binary sorted temporal index from
 * 	binary travel time matrix that can be used later more quickly
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
#include <thread>

#define USE_MUTEX
#include "ttimes2.h"

template<class index_t>
bool do_check(tt_sorted_index<index_t>& ttindex, uint8_t min_hour2, uint8_t max_hour2) {
	for(uint8_t h=min_hour2;h<max_hour2;h++) {
		for(unsigned int i=0;i<ttindex.get_matrix_size();i++) {
			const index_t* i1 = ttindex.get_sorted_ids_from(h,i);
			if(!i1) {
				fprintf(stderr,"Could not get sorted travel times!\n");
				return false;
			}
			uint16_t last_time = 0;
			for(index_t j=0;j<ttindex.get_matrix_size();j++) {
				index_t j1 = i1[j];
				uint16_t tt = ttindex.get_travel_time_id(i,j1,h);
				if(tt < last_time) {
					fprintf(stderr,"Travel times not sorted for hour %u: %u -- %u!\n",(unsigned int)h,i,(unsigned int)j1);
					return false;
				}
			}
			
			i1 = ttindex.get_sorted_ids_to(h,i);
			if(!i1) {
				fprintf(stderr,"Could not get sorted travel times!\n");
				return false;
			}
			last_time = 0;
			for(index_t j=0;j<ttindex.get_matrix_size();j++) {
				index_t j1 = i1[j];
				uint16_t tt = ttindex.get_travel_time_id(j1,i,h);
				if(tt < last_time) {
					fprintf(stderr,"Travel times not sorted for hour %u: %u -- %u!\n",(unsigned int)h,(unsigned int)j1,i);
					return false;
				}
			}
		}
	}
	return true;
}

template<class index_t>
int do_run(tt_sorted_index<index_t>& ttindex, const char* fnbase, const char* fnext, 
		const char* fnbase_out, const char* fnext_out, unsigned int min_hour, unsigned int max_hour, int nthreads, bool check) {
	
	ttindex.open_files(fnbase,fnext);
	
	uint8_t min_hour2 = (uint8_t)min_hour;
	uint8_t max_hour2 = (uint8_t)max_hour;
	
	bool res;
	if(check) {
		ttindex.open_sorted_files(fnbase_out, fnext_out);
		res = do_check(ttindex, min_hour2, max_hour2);
		if(res) fprintf(stdout,"Sorted checks OK\n");
	}
	else {
		res = ttindex.save_sorted_files(fnbase_out, fnext_out, min_hour2, max_hour2, nthreads);
		if(!res) fprintf(stderr,"Error saving files!\n");
	}
	
	return !res;
}

int main(int argc, char **argv)
{
	char* fnbase = 0; /* base filename for travel times index */
	char fnext1[] = ".bin";
	char* fnext = fnext1;
	
	char* fnbase_out = 0; /* base filename for output */
	char fnext2[] = ".bin";
	char* fnext_out = fnext2;
	unsigned int matrix_size = 0;
	
	bool check = false; /* if true, read and check sorted index instead of creating it */
	
	unsigned int min_hour = 0;
	unsigned int max_hour = 24;
	
	bool use_32bit_index = false;
	
	int nthreads = std::thread::hardware_concurrency();
	
	for(int i=1;i<argc;i++) {
		if(argv[i][0] == '-') switch(argv[i][1]) {
			case 'b':
				fnbase = argv[i+1];
				i++;
				break;
			case 'o':
				fnbase_out = argv[i+1];
				i++;
				break;
			case 'e':
				fnext = argv[i+1];
				i++;
				break;
			case 'E':
				fnext_out = argv[i+1];
				i++;
				break;
			case 'c':
				check = true;
				break;
			case 'M':
				matrix_size = atoi(argv[i+1]);
				i++;
				break;
			case 'h':
				min_hour = atoi(argv[i+1]);
				max_hour = atoi(argv[i+2]);
				i+=2;
				break;
			case 'T':
				nthreads = atoi(argv[i+1]);
				i++;
				break;
			case '3':
				use_32bit_index = true;
				break;
			default:
				fprintf(stderr,"Unknown parameter: %s\n",argv[i]);
				break;
		}
		else fprintf(stderr,"Unknown parameter: %s\n",argv[i]);
	}
	
	if( ! (fnbase && fnbase_out) ) {
		fprintf(stderr,"Missing filenames!\n");
		return 1;
	}
	
	if(min_hour >= max_hour || min_hour >= 24 || max_hour > 24) {
		fprintf(stderr,"Invalid minimum / maximum hour given!\n");
		return 1;
	}
	
	if(matrix_size) {
		if(use_32bit_index) {
			tt_sorted_index<uint32_t> ttindex(matrix_size);
			return do_run(ttindex, fnbase, fnext, fnbase_out, fnext_out, min_hour, max_hour, nthreads, check);
		}
		else {
			tt_sorted_index<uint16_t> ttindex(matrix_size);
			return do_run(ttindex, fnbase, fnext, fnbase_out, fnext_out, min_hour, max_hour, nthreads, check);
		}
	}
	else {
		if(use_32bit_index) {
			tt_sorted_index<uint32_t> ttindex;
			return do_run(ttindex, fnbase, fnext, fnbase_out, fnext_out, min_hour, max_hour, nthreads, check);
		}
		else {
			tt_sorted_index<uint16_t> ttindex;
			return do_run(ttindex, fnbase, fnext, fnbase_out, fnext_out, min_hour, max_hour, nthreads, check);
		}
	}
}

