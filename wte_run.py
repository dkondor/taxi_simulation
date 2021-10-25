#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#  wte_run.py -- test program for the python simulation interface
#  
#  Copyright 2021 Daniel Kondor <kondor.dani@gmail.com>
#  
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are
#  met:
#  
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
#  * Neither the name of the  nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  
#  

import wte, sys, getopt

def main(args):
	matrix_dir = "data/nyc_travel_times"
	matrix_size = 4092
	tripsfile = "data/nytrips/nytrips_15180.bz2"
	ndrivers = 5000
	seed = 1
	twait = 300
	outfn = None
	pr = 1.0
	cruising_dist = None
	cruising_mult = False
	
	opts, args = getopt.getopt(args[1:], 't:i:M:D:r:s:T:m:o:C:c')
	
	for opt, arg in opts:
	#	print("opt: {}, arg: {}".format(opt, arg))
		if opt == '-t':
			tripsfile = arg
		elif opt == 'i':
			matrix_dir = arg
		elif opt == '-M':
			matrix_size = int(arg)
		elif opt == '-D':
			ndrivers = int(arg)
		elif opt == '-r':
			pr = float(arg)
		elif opt == '-s':
			seed = int(arg)
		elif opt == '-T':
			twait = int(arg)
		elif opt == '-o':
			outfn = arg
		elif opt == '-C':
			cruising_dist = arg
		elif opt == '-c':
			cruising_mult = True
			
	
	w1 = wte.wait_time_estimator()
	w1.set_seed(seed)
	w1.load_index_default_fn(matrix_size,matrix_dir,cruising_dist is not None)
	w1.read_trips_csv(tripsfile, 'bz2')
	w1.add_drivers(ndrivers,False)
	
	if cruising_dist is not None:
		w1.strategy = wte.wait_time_estimator.cruising_strategy.DRIVER_RANDOM_NODE
		if cruising_mult:
			w1.read_cruising_target_dist_mult(cruising_dist)
		else:
			w1.read_cruising_target_dist(0, cruising_dist)
	
	with open(outfn, "w") as out:
		while not w1.is_end:
			if w1.process_next_trip(twait):
				res = w1.get_last_res()
				out.write("{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(res['tp'],res['np'],res['nreject'],res['trip_id'],
														res['driver_id'],res['tw'],res['ndrivers'],res['tt'],res['di']))
		
			
	return 0

if __name__ == '__main__':
	import sys
	sys.exit(main(sys.argv))

