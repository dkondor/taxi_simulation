#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#  libwte.py -- interface to C++ library to simulate taxi trips and
#	estimate passenger waiting times
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


import sys
from ctypes import *
from enum import Enum

this = sys.modules[__name__]
this._libwte_path = "./libwte.so"

class discrete_trip(Structure):
	_fields_ = [("id", c_ulong), # trip ID
				("start_ts", c_uint), # start time (UNIX timestamp)
				("end_ts", c_uint), # end time
				("start_node", c_uint), # start node ID
				("end_node", c_uint)] # end node ID
	
	def __getitem__(self, key):
		return getattr(self, key)
	
	def __setitem__(self, key, value):
		return setattr(self, key, value)

class last_res(Structure):
	_fields_ = [("tp", c_uint), # current simulation time
				("np", c_uint), # number of trips processed (including rejected trips)
				("trip_id", c_ulong), # trip ID (of last served trip)
				("driver_id", c_uint), # driver ID assigned to the last trip
				("nreject", c_uint), # number of rejected trips
				("tw", c_uint), # waiting time of last served trip (in seconds)
				("ndrivers", c_uint), # total number of idle drivers (including moving drivers)
				("tt", c_uint), # travel time of last terved trip ( = end_ts - start_ts)
				("di", c_uint), # idle time of driver assigned to the last trip (before serving the trip; in seconds)
				("trip_dist", c_uint), # distance of the last served trip (in meters)
				("driver_dist", c_uint)] # total empty travel of the driver assigned to the last served trip since their last trip (in meters)
	
	def __getitem__(self, key):
		return getattr(self, key)
	
	def __setitem__(self, key, value):
		return setattr(self, key, value)

class driver_summary(Structure):
	_fields_ = [("trips_served", c_uint), # total number of trips served by this driver
				("start_time", c_uint), # start time of the first trip served by this driver
				("end_time", c_uint), # end time of the last trip served by this driver
				("total_trip_time", c_uint), # total time spent serving trips so far (in seconds)
				("total_empty_time", c_uint), # total time spent empty (either idle, cruising or driving to pick up a passenger)
				("total_trip_distance", c_uint), # total distance of trips served (in meters)
				("total_empty_distance", c_uint), # total distance spent empty (either cruising or driving to pick up a passenger); does not include any cruising before the first trip or since the last trip
				("last_empty_distance", c_uint), # distance spent cruising since the end of the last trip
				("st", c_int)] # current state of the driver -- enum
	
	class state(Enum):
		OCCUPIED = 0 # driver is serving a trip (or is traveling to serve a trip)
		ADDED = 1 # initial state of a driver before becomes available
		IDLE = 2 # driver is idle (at a node, not moving)
		MOVING = 3 # driver is repositioning / cruising
		REMOVED = 4 # driver was removed, ID is not valid anymore
	
	def __getitem__(self, key):
		res = getattr(self, key)
		if key == "st":
			return driver_summary.state(res)
		return res
	
	def __setitem__(self, key, value):
		return setattr(self, key, value)


class libwte1:
	def __init__(self,path):
		self.libwte = cdll.LoadLibrary(path)
		
		# wte -- taxi simulation in real space (travel times loaded from binary index files))
		self.wte_new = self.libwte.wte_new
		self.wte_new.restype = c_void_p
		self.wte_free = self.libwte.wte_free
		self.wte_free.argtypes = [c_void_p]
		
		self.wte_load_index = self.libwte.wte_load_index
		self.wte_load_distances = self.libwte.wte_load_distances
		self.wte_process_trip = self.libwte.wte_process_trip
		self.wte_reset = self.libwte.wte_reset
		self.wte_clear = self.libwte.wte_clear
		self.wte_add_driver = self.libwte.wte_add_driver
		
		self.wte_load_index.argtypes = [c_void_p, c_uint, c_char_p, c_char_p, c_char_p, c_char_p, c_char_p, c_char_p, c_int]
		self.wte_load_index.restype = c_int
		self.wte_load_distances.argtypes = [c_void_p, c_char_p, c_int]
		self.wte_load_distances.restype = c_int
		self.wte_process_trip.argtypes = [c_void_p, POINTER(discrete_trip), c_uint, c_int, POINTER(c_int)]
		self.wte_process_trip.restpye = c_int
		
		self.wte_reset.argtypes = [c_void_p, c_int];
		self.wte_clear.argtypes = [c_void_p];
		self.wte_reset.restype = c_int;
		self.wte_clear.restype = c_int;
		
		self.wte_add_driver = self.libwte.wte_add_driver
		self.wte_add_driver.argtypes = [c_void_p, c_uint, c_uint]
		self.wte_add_driver.restype = c_int
		
		self.wte_get_last_res = self.libwte.wte_get_last_result
		self.wte_get_last_res.argtypes = [c_void_p]
		self.wte_get_last_res.restype = POINTER(last_res)
		
		self.wte_set_seed = self.libwte.wte_set_seed
		self.wte_set_seed.argtypes = [c_void_p, c_ulong]
		
		self.wte_read_trips = self.libwte.wte_read_trips
		self.wte_read_trips.argtypes = [c_void_p, c_char_p]
		self.wte_read_trips.restype = c_int
		self.wte_read_trips_csv = self.libwte.wte_read_trips_csv
		self.wte_read_trips_csv.argtypes = [c_void_p, c_char_p, c_char_p, c_uint, c_uint, c_int, c_uint, c_uint, c_uint, c_uint, c_double, c_double, c_int]
		self.wte_read_trips_csv.restype = c_int
		
		self.wte_add_trip = self.libwte.wte_add_trip
		self.wte_add_trip.argtypes = [c_void_p, POINTER(discrete_trip)]
		self.wte_add_trip.restype = c_int
		self.wte_sort_trips = self.libwte.wte_sort_trips
		self.wte_sort_trips.argtypes = [c_void_p, c_int]
		self.wte_is_sorted = self.libwte.wte_is_sorted
		self.wte_is_sorted.argtypes = [c_void_p]
		self.wte_is_sorted.restype = c_int
		
		self.wte_add_drivers = self.libwte.wte_add_drivers
		self.wte_add_drivers.argtypes = [c_void_p, c_ulong, c_double, c_int]
		self.wte_add_drivers.restype = c_int
		
		self.wte_process_next_trip = self.libwte.wte_process_next_trip
		self.wte_process_next_trip.argtypes = [c_void_p, c_uint, POINTER(c_int)]
		self.wte_process_next_trip.restype = c_int
		
		self.wte_is_end = self.libwte.wte_is_end
		self.wte_is_end.argtypes = [c_void_p]
		self.wte_is_end.restype = c_int
		
		self.wte_get_trip = self.libwte.wte_get_trip
		self.wte_get_trip.argtypes = [c_void_p, c_ulong]
		self.wte_get_trip.restype = POINTER(discrete_trip)
		self.wte_ntrips = self.libwte.wte_ntrips
		self.wte_ntrips.argtypes = [c_void_p]
		self.wte_ntrips.restype = c_ulong
		
		self.wte_get_error = self.libwte.wte_get_error
		self.wte_get_error.argtypes = [c_void_p]
		self.wte_get_error.restype = c_char_p
		
		self.wte_set_handle_exceptions = self.libwte.wte_set_handle_exceptions
		self.wte_set_handle_exceptions.argtypes = [c_void_p, c_int]
		self.wte_get_handle_exceptions = self.libwte.wte_get_handle_exceptions
		self.wte_get_handle_exceptions.argtypes = [c_void_p]
		self.wte_get_handle_exceptions.restype = c_int
		
		self.wte_get_trip_idx = self.libwte.wte_get_trip_idx
		self.wte_get_trip_idx.argtypes = [c_void_p]
		self.wte_get_trip_idx.restype = c_ulong
		
		# driver statistics
		self.wte_get_total_drivers = self.libwte.wte_get_total_drivers
		self.wte_get_total_drivers.argtypes = [c_void_p]
		self.wte_get_total_drivers.restype = c_uint
		self.wte_get_total_driver_ids = self.libwte.wte_get_total_driver_ids
		self.wte_get_total_driver_ids.argtypes = [c_void_p]
		self.wte_get_total_driver_ids.restype = c_uint
		self.wte_get_driver_info = self.libwte.wte_get_driver_info
		self.wte_get_driver_info.argtypes = [c_void_p, c_uint]
		self.wte_get_driver_info.restype = POINTER(driver_summary)
		
		# cruising
		self.wte_get_cruising_strategy = self.libwte.wte_get_cruising_strategy
		self.wte_get_cruising_strategy.argtypes = [c_void_p]
		self.wte_get_cruising_strategy.restype = c_int
		self.wte_set_cruising_strategy = self.libwte.wte_set_cruising_strategy
		self.wte_set_cruising_strategy.argtypes = [c_void_p, c_int]
		
		self.wte_read_cruising_target_dist = self.libwte.wte_read_cruising_target_dist
		self.wte_read_cruising_target_dist.argtypes = [c_void_p, c_uint, c_char_p]
		self.wte_read_cruising_target_dist.restype = c_int
		
		self.wte_read_cruising_target_dist_mult = self.libwte.wte_read_cruising_target_dist_mult
		self.wte_read_cruising_target_dist_mult.argtypes = [c_void_p, c_char_p]
		self.wte_read_cruising_target_dist_mult.restype = c_int
		
		self.wte_read_road_network = self.libwte.wte_read_road_network
		self.wte_read_road_network.argtypes = [c_void_p, c_char_p, c_int, c_int]
		self.wte_read_road_network.restype = c_int
		
		self.wte_get_random_walk_max_dist = self.libwte.wte_get_random_walk_max_dist
		self.wte_get_random_walk_max_dist.argtypes = [c_void_p]
		self.wte_get_random_walk_max_dist.restype = c_uint
		
		self.wte_set_random_walk_max_dist = self.libwte.wte_set_random_walk_max_dist
		self.wte_set_random_walk_max_dist.argtypes = [c_void_p, c_uint]
		
		# filtering and copy of trips
		self.wte_filter_trips = self.libwte.wte_filter_trips
		self.wte_filter_trips.argtypes = [c_void_p, POINTER(c_ulong), c_ulong]
		
		self.wte_copy_trips_filter = self.libwte.wte_copy_trips_filter
		self.wte_copy_trips_filter.argtypes = [c_void_p, POINTER(c_ulong), c_ulong]
		self.wte_copy_trips_filter.restype = c_void_p
		
		
		# get info about idle and moving drivers
		self.wte_n_idle_drivers = self.libwte.wte_n_idle_drivers
		self.wte_n_idle_drivers.argtypes = [c_void_p]
		self.wte_n_idle_drivers.restype = c_uint
		
		self.wte_n_moving_drivers = self.libwte.wte_n_moving_drivers
		self.wte_n_moving_drivers.argtypes = [c_void_p]
		self.wte_n_moving_drivers.restype = c_uint
		
		self.wte_idle_drivers_start = self.libwte.wte_idle_drivers_start
		self.wte_idle_drivers_start.argtypes = [c_void_p]
		
		self.wte_idle_drivers_is_end = self.libwte.wte_idle_drivers_is_end
		self.wte_idle_drivers_is_end.argtypes = [c_void_p]
		self.wte_idle_drivers_is_end.restype = c_int
		
		self.wte_idle_drivers_advance = self.libwte.wte_idle_drivers_advance
		self.wte_idle_drivers_advance.argtypes = [c_void_p]
		self.wte_idle_drivers_advance.restype = c_int
		
		self.wte_idle_drivers_current = self.libwte.wte_idle_drivers_current
		self.wte_idle_drivers_current.argtypes = [c_void_p, POINTER(c_uint), POINTER(c_uint)]
		self.wte_idle_drivers_current.restype = c_int
		
	
		self.wte_moving_drivers_start = self.libwte.wte_moving_drivers_start
		self.wte_moving_drivers_start.argtypes = [c_void_p]
		
		self.wte_moving_drivers_is_end = self.libwte.wte_moving_drivers_is_end
		self.wte_moving_drivers_is_end.argtypes = [c_void_p]
		self.wte_moving_drivers_is_end.restype = c_int
		
		self.wte_moving_drivers_advance = self.libwte.wte_moving_drivers_advance
		self.wte_moving_drivers_advance.argtypes = [c_void_p]
		self.wte_moving_drivers_advance.restype = c_int
		
		self.wte_moving_drivers_current = self.libwte.wte_moving_drivers_current
		self.wte_moving_drivers_current.argtypes = [c_void_p, POINTER(c_uint), POINTER(c_uint)]
		self.wte_moving_drivers_current.restype = c_int
		
		
		self.wte_get_driver_max_idle = self.libwte.wte_get_driver_max_idle
		self.wte_get_driver_max_idle.argtypes = [c_void_p]
		self.wte_get_driver_max_idle.restype = c_uint
		self.wte_set_driver_max_idle = self.libwte.wte_set_driver_max_idle
		self.wte_set_driver_max_idle.argtypes = [c_void_p, c_uint]
		
		
		self.wte_prune_drivers = self.libwte.wte_prune_drivers
		self.wte_prune_drivers.argtypes = [c_void_p]
		self.wte_prune_drivers.restype = c_int
		
		self.wte_remove_driver = self.libwte.wte_remove_driver
		self.wte_remove_driver.argtypes = [c_void_p, c_uint]
		self.wte_remove_driver.restype = c_int
		
		
		self.wte_reposition_driver = self.libwte.wte_reposition_driver
		self.wte_reposition_driver.argtypes = [c_void_p, c_uint, c_uint]
		self.wte_reposition_driver.restype = c_int
		
		self.cruising_cb = CFUNCTYPE(c_uint, c_uint, c_uint)
		self.wte_set_cruising_callback = self.libwte.wte_set_cruising_callback
		self.wte_set_cruising_callback.argtypes = [c_void_p, self.cruising_cb]
		
		self.cruising_data_cb = CFUNCTYPE(c_uint, c_uint, c_uint, c_void_p)
		self.wte_set_cruising_callback_with_data = self.libwte.wte_set_cruising_callback_with_data
		self.wte_set_cruising_callback_with_data.argtypes = [c_void_p, self.cruising_data_cb, c_void_p]
		

_libwte = None

def load_wte_lib(path):
	"""Load the C++ shared library needed by this module.
	(only needs to be called if the library is not in the same directory as this module)"""
	this._libwte = libwte1(path)


class wait_time_estimator:
	"""Class interface for C++ taxi simulation code"""
	class driver_info:
		def __init__(self, w):
			self.w = w
		
		def __getitem__(self, key):
			if type(key) is not int:
				raise TypeError("Drivers must be indexed with integers!")
			if key >= _libwte.wte_get_total_driver_ids(self.w):
				raise IndexError("Driver ID too large")
			x = _libwte.wte_get_driver_info(self.w, key)
			if x == 0 or x is None:
				x = c_char_p(_libwte.wte_get_error(self.w)).value
				if x is None:
					raise BaseException("Unknown error!\n")
				else:
					raise BaseException(x.decode('utf-8'))
			return x.contents
		
		def __len__(self):
			return _libwte.wte_get_total_driver_ids(self.w)


	def __init__(self, wte_ptr = None):
		"""
			Create a new instance of this class.
		"""
		if _libwte is None:
		 	load_wte_lib(_libwte_path)
		if (type(wte_ptr) is c_void_p):
			self.w = wte_ptr.value
		elif wte_ptr is None:
			self.w = _libwte.wte_new()
		else:
			raise BaseException("Invalid parameter for pointer: {}!\n".format(type(wte_ptr)))
		if self.w == 0 or self.w is None:
			raise BaseException("Could not allocate memory for wait_time_estimator class!\n")
		self.di = wait_time_estimator.driver_info(self.w)
	
	def __del__(self):
		"""Destructor (needed to free up memory used)"""
		if self.w != 0 and self.w is not None:
			_libwte.wte_free(self.w)
			self.w = 0
	
	def get_error(self):
		"""Return a string description of the last error occured (if any) or None"""
		x = c_char_p(_libwte.wte_get_error(self.w))
		return x.value
	
	def handle_error(self):
		"""Throw an exception with the description of the last error that occured"""
		x = self.get_error()
		if x is None:
			raise BaseException("Unknown error!\n")
		else:
			raise BaseException(x.decode('utf-8'))
	
	# load travel time index -- needed to know which driver can reach which passenger
	def load_index(self, matrix_size, fnbase, fnext, fnbase_sorted = None, fnext_sorted = None,
		fnbase_paths = None, fnext_paths = None, preread = False):
		"""
			Load the (binary) travel time matrix and (optionally) the associated indexes.
			This is necessary before running any simulations.
			
			Parameters:
				matrix_size: size of the matrix (number of nodes in the city), needs to be given as a parameter;
					trips can include node IDs between zero and matrix_size-1 (inclusive)
				fnbase: base filename of travel time matrix binary files
				fnext: extension of travel time matrix binary files (including '.');
					filenames are expected as "fnbase[0-23]fnext"
					(24 files in total, corresponding to each hour in the day)
				fnbase_sorted: base filename of the sorted index binary files (optional, if this is None,
					these will be generated when needed, but this can add a significant additional runtime)
				fnext_sorted: extension of sorted index binary files (including '.')
				fnbase_path: base filename for the shortest path index matrix (only needed if using cruising)
				fnext_sorted: extension of path index matrix (including '.')
				preread: if True, try to ensure that all of the index files are read into memory
		"""
		if fnbase_sorted is not None:
			fnbase_sorted = bytes(fnbase_sorted,'utf-8')
		if fnext_sorted is not None:
			fnext_sorted = bytes(fnext_sorted,'utf-8')
		if fnbase_paths is not None:
			fnbase_paths = bytes(fnbase_paths,'utf-8')
		if fnext_paths is not None:
			fnext_paths = bytes(fnext_paths,'utf-8')
		tmp = 1 if preread else 0
		res = _libwte.wte_load_index(self.w, matrix_size, bytes(fnbase,'utf-8'), bytes(fnext,'utf-8'),
			fnbase_sorted, fnext_sorted, fnbase_paths, fnext_paths, tmp)
		if res == 0:
			self.handle_error()
	
	# load all indices with the default filenames
	def load_index_default_fn(self, matrix_size, dir_name, use_paths = False, preread = False):
		"""
			Load the (binary) travel time matrix and (optionally) the associated indexes.
			This is necessary before running any simulations.
			Use the default filenames (Ho, index and paths) inside the given directory)
			
			Parameters:
				matrix_size: size of the matrix (number of nodes in the city), needs to be given as a parameter;
					trips can include node IDs between zero and matrix_size-1
				dir_name: location of the index files
				use_path: if True, attempt to load the shorted path index as well (only needed if using cruising)
				preread: if True, try to ensure that all of the index files are read into memory
		"""
		return self.load_index(matrix_size, dir_name + '/Ho', '.bin', dir_name + '/index', '.bin',
			dir_name + '/paths' if use_paths else None, '.bin' if use_paths else None, preread)
	
	# load distances between nodes
	def load_distances(self, fn, preread = False):
		"""
			Load the (binary) travel distance matrix from the given file. This is only necessary if travel distances are of interest.
				preread: if True, try to ensure that all of the index files are read into memory
		"""
		tmp = 1 if preread else 0
		res = _libwte.wte_load_distances(self.w, bytes(fn,'utf-8'), tmp)
		if res == 0:
			self.handle_error()
	
	# process one trip given as a parameter
	def process_trip(self, t, tmax1 = 0, allow_moving = True):
		"""
			Process one trip request, given as the t parameter, trying to match it to an available driver.
			
			Parameters:
				t: trip, either the return value of the get_trip() or random_trip() function, or a dict, containing the members
					"id" (64-bit unsigned int), "start_ts", "end_ts", "start_node" and "end_node" (all 32-bit unsigned ints)
					note: all node IDs have to be less than the matrix_size parameter given to load_index()
				tmax1: maximum acceptable waiting time for this passenger (in seconds; zero means unlimited)
				allow_moving: whether currently repositioning drivers are allowed to be assigned to this trip as well
				
			Returns:
				True if a driver was successfully found; in this case, the get_last_res() function can be
					used to get relevant info
				False if no match was found (no drivers available or waiting time would be too large)
				
			Note:
				Trip start time cannot be before than the start time of the last trip that was processed (this will
				result in an exception), so trips have to be supplied in time order. The reset() function can be used
				to start over from the beginning
		"""
		res1 = c_int(0)
		tmp1 = 1 if allow_moving else 0
		if (type(t) is c_void_p) or (type(t) is int) or (type(t) is POINTER(discrete_trip)):
			res = _libwte.wte_process_trip(self.w, t, tmax1, tmp1, byref(res1))
		elif type(t) is discrete_trip:
			res = _libwte.wte_process_trip(self.w, byref(t), tmax1, tmp1, byref(res1))
		else:
			trip = discrete_trip(t['id'], t['start_ts'], t['end_ts'], t['start_node'], t['end_node'])
			res = _libwte.wte_process_trip(self.w, byref(trip), tmax1, tmp1, byref(res1))
		if res == 0:
			self.handle_error()
		if res1.value == 0:
			return False
		return True
		
	# reset simulation to the beginning of the day and erase all drivers
	def reset(self, clear_drivers = False):
		"""
			Reset the state of simulation, including erasing all drivers (have to use add_drivers() again)
		"""
		cd = 1 if clear_drivers else 0
		res = _libwte.wte_reset(self.w, cd)
		if res == 0:
			self.handle_error()
	
	#  reset simulation to the beginning of the day and erase all drivers + clear all trips read previously
	def clear(self):
		"""
			Reset the state, including eraseing all drivers and all trips stored in this instance
		"""
		res = _libwte.wte_clear(self.w)
		if res == 0:
			self.handle_error()
	
	# add one driver at the given node with the given start time
	def add_driver(self,start_time,node = 0):
		"""
			Add one driver to the system, at the given node and start time.
		"""
		res = _libwte.wte_add_driver(self.w,node,start_time)
		if res == 0:
			self.handle_error()
	
	def get_last_res(self):
		 tmp = _libwte.wte_get_last_res(self.w)
		 return tmp.contents
	
	def set_seed(self,seed):
		"""
			Set seed for the random number generator used.
		"""
		_libwte.wte_set_seed(self.w,seed)
	
	# read a list of trips to use
	def read_trips(self,filename):
		"""
			Read a list of taxi trips to use as input (demand) for the simulation.
		"""
		res = _libwte.wte_read_trips(self.w,bytes(filename,'utf-8'))
		if res == 0:
			self.handle_error()
			
	# read a list of trips to use
	def read_trips_csv(self,filename,compress = None, tmin = None, tmax = None, tlimit_in_day = False,
			length_min = None, length_max = None, dist_min = None, dist_max = None, speed_min = None, speed_max = None,
			clear_trips = False, chicago_format = False, node_filter_fn = None):
		"""
			Read a list of taxi trips to use as input (demand) for the simulation.
			Use the csv format, which has the following differences from the previous one:
			  (1) order of columns is changes;
			  (2) instead of trip ID, we have driver ID, this is replaced
			  (3) timestamps are real ones, we replace them to limit to one day
			
			Parameters:
				filename: file to open
				compress: None for uncompressed input or type of compression for compressed one
					supported values: "gz", "gzip", "bz", "bz2", "bzip2", "xz", "lzma"
				tmin: if not None, minimum start time to filter trips (inclusive)
				tmax: if not None, maximum start time to filter trips (inclusive)
				tlimit_in_day: if True, tmin and tmax are interpreted as times within a day (i.e. seconds between 0 and 86400);
					otherwise, they are interpreted as absolute timestamps
				length_min: if not None, trip duration must be longer than this (in seconds)
				length_max: if not None, filter out trips longer than this (in seconds)
				dist_min: if not None, filter out trips with less distance (in meters, based on shortest path, load_distances() must be called first)
				dist_max: if not None, filter out trips with longer distance (in meters, based on shortest path, load_distances() must be called first)
				speed_min: if not None, filter out trips with lower average speed (in meters / second, distance is based on shortest path, load_distances() must be called first)
				speed_max: if not None, filter out trips with faster average speed (in meters / second, distance is based on shortest path, load_distances() must be called first)
				clear_trips: if True, any previously loaded trips are erased first; otherwise, newly read trips are
					added along any previously read trips
				chicago_format: if True, the file format is the one used for the Chicago trips (no driver ID, separate travel time, start time is 15-minute precision, a random time is added)
				node_filter_fn: if not None, a list of nodes is read from this file and trips are only kept if the start and end nodes are in this set
				
				Note: if any of the conditions are given, trips that are read are filtered:
					trips with parameters outside the given ranges are discarded.
					If only one end of a range is given, the other end is unlimited.
					Trips with zero duration are always discarded.
				
				Example: read trips from 3AM to 3AM on the next day:
				w1.read_trips_csv('day1.csv', None, 10800, None, True, clear_trips = True)
				w1.read_trips_csv('day2.csv', None, None, 10799, True, clear_trips = False)
		"""
		res = _libwte.wte_read_trips_csv(self.w,bytes(filename,'utf-8'),
			bytes(compress,'utf-8') if compress is not None else None,
			tmin if tmin is not None else 0, tmax if tmax is not None else 0,
			1 if tlimit_in_day else 0,
			length_min if length_min is not None else 0,
			length_max if length_max is not None else 0,
			dist_min if dist_min is not None else 0,
			dist_max if dist_max is not None else 0,
			speed_min if speed_min is not None else 0,
			speed_max if speed_max is not None else 0,
			1 if clear_trips else 0,
			bytes(node_filter_fn, 'utf-8') if node_filter_fn is not None else None,
			1 if chicago_format else 0)
		if res == 0:
			self.handle_error()
	
	# add one trip to the internal list
	def add_trip(self, t):
		"""
			Add a trip supplied in t to the internal list of trips.
		"""
		if (type(t) is c_void_p) or (type(t) is int) or (type(t) is POINTER(discrete_trip)):
			res = _libwte.wte_add_trip(self.w, t)
		elif type(t) is discrete_trip:
			res = _libwte.wte_add_trip(self.w, byref(t))
		else:
			trip = discrete_trip(t['id'],t['start_ts'],t['end_ts'],t['start_node'],t['end_node'])
			res = _libwte.wte_add_trip(self.w, byref(trip))
		if res == 0:
			self.handle_error()
	
	# ensure that the trips are sorted by time
	def sort_trips(self, reassign_ids = True):
		"""
			Sort internally stored trips by start time.
		"""
		_libwte.wte_sort_trips(self.w, 1 if reassign_ids else 0)
	
	@property
	def sorted(self):
		"""
			True if trips are already sorted by start time.
		"""
		res = _libwte.wte_is_sorted(self.w)
		return False if res == 0 else True
	
	# add drivers in random locations
	def add_drivers(self, ndrivers, tavg = None, use_end_locations = False):
		"""
			Add a set of drivers -- locations are determined based on trip start or end locations.
			
			Drivers added by this function are not available immediately, but are assumed to finish their
			previous trips at random times (that is determined by the average length of trips).
			
			Parameters:
				ndrivers: number of drivers to add
				use_end_locations: if True, trip end locations are sampled randomly to place the drivers initially;
					if False, trip start locations are sampled.
		"""
		use_end_locations2 = 1 if use_end_locations else 0
		if tavg is None:
			tavg = -1.0
		res = _libwte.wte_add_drivers(self.w,ndrivers,tavg,use_end_locations2)
		if res == 0:
			self.handle_error()
		
	def process_next_trip(self,tmax1=0):
		"""
			Process the next trip from the list of trips previously loaded.
			
			First, the trip is kept with pr probability and discarded with 1-pr probability. This process is repeated
			until a trip is found to be kept.
			
			Next, a maximum acceptable waiting time is selected for the passengers (if tmax1 == 0). Then a driver is
			attempted to be assigned, using process_trip().
			
			Consecutive calls of this function process the list of trips sequentially. Use read_trips() to read the
			trips to use and reset() to restart from the beginning.
			
			Parameters:
				tmax1: maximum acceptable waiting time for the passenger (if > 0). If tmax1 == 0, a random value is
					selected from a gamma distribution.
		"""
		res1 = c_int(0)
		res = _libwte.wte_process_next_trip(self.w,tmax1,byref(res1))
		if res == 0:
			self.handle_error()
		if res1.value == 0:
			return False
		return True
	
	@property
	def is_end(self):
		"""
			Returns True if all trips were already processed.
		"""
		res = _libwte.wte_is_end(self.w)
		if res == 0:
			return False
		return True
	
	@property
	def ntrips(self):
		"""
			Returns the number of taxi trips stored in this instance (read with read_trips()).
		"""
		return _libwte.wte_ntrips(self.w)
	
	@property
	def trip_idx(self):
		"""
			Returns the index of the current trip (the one to be processed next with process_next_trip()).
		"""
		return _libwte.wte_get_trip_idx(self.w)
	
	@property
	def ndrivers(self):
		"""
			Number of total active (i.e. non-removed) drivers. Note that this
			will not be equal to len(drivers) if some drivers were removed.
		"""
		return _libwte.wte_get_total_drivers(self.w)
	
	@property
	def drivers(self):
		"""
			List of drivers created over the course of the simulation.
			Includes drivers who were already removed as well.
		"""
		return self.di
	
	def get_trip(self,idx):
		"""
			Returns the trip at the given index as a discrete_trip object (can be used as a dictionary as well).
		"""
		t = _libwte.wte_get_trip(self.w,idx)
		if t is None:
			self.handle_error()
		return t.contents
	
	def get_trip_p(self,idx):
		"""
			Returns the trip at the given index as an internal handle (to be used by process_trip()).
			
			Note that this is invalidated by a call to read_trips() or clear(), so the handle should not be used after that.
		"""
		t = _libwte.wte_get_trip(self.w,idx)
		if t is None:
			self.handle_error()
		return t
	
	def get_idle_drivers(self, include_repositioning = True):
		"""
			Get a list of currently idle drivers as tuples of (driver_id, node_id).
			
			Parameters:
				include_repositioning: whether to include drivers that are in the process of repositioning
		"""
		res = []
		_libwte.wte_idle_drivers_start(self.w)
		while _libwte.wte_idle_drivers_is_end(self.w) == 0:
			driver_id = c_uint(0)
			node_id = c_uint(0)
			if _libwte.wte_idle_drivers_current(self.w, byref(driver_id), byref(node_id)) == 0:
				self.handle_error()
			res.append((driver_id.value, node_id.value))
			_libwte.wte_idle_drivers_advance(self.w)
		if include_repositioning:
			_libwte.wte_moving_drivers_start(self.w)
			while _libwte.wte_moving_drivers_is_end(self.w) == 0:
				driver_id = c_uint(0)
				node_id = c_uint(0)
				if _libwte.wte_moving_drivers_current(self.w, byref(driver_id), byref(node_id)) == 0:
					self.handle_error()
				res.append((driver_id.value, node_id.value))
				_libwte.wte_moving_drivers_advance(self.w)
		return res
	
	
	@property
	def n_idle_drivers(self):
		"""
			Returns the number of currently idle (non-moving) drivers.
		"""
		return _libwte.wte_n_idle_drivers(self.w)
	
	@property
	def n_moving_drivers(self):
		"""
			Returns the number of currently moving (i.e. repositioning) drivers.
		"""
		return _libwte.wte_n_moving_drivers(self.w)
		
	def reposition_driver(self, driver_id, target):
		"""
			Start repositioning the given driver (identified with its ID) toward the
			target node. The given driver needs to be idle or currently repositioning.
			If the drivers is busy (i.e. serving a trip, or is on their way to pickup
			a passenger), or if the driver does not exist, an exception is raised.
			
			Parameters:
				driver_id: the ID of the driver to reposition; this would typically come
					the list returned by get_idle_drivers()
				target: node ID of the target where this driver should reposition to
		"""
		if _libwte.wte_reposition_driver(self.w, driver_id, target) == 0:
			self.handle_error()
	
	def set_cruising_callback(self, cb):
		"""
			Set a callback function that is called whenever a driver finishes a trip.
			This function must return the target node ID where the driver should
			reposition themselves. The function is given two arguments, the first
			is the current node ID, the second is the current time.
		"""
		self.last_cb = _libwte.cruising_cb(cb)
		_libwte.wte_set_cruising_callback(self.w, self.last_cb)
	
	def set_cruising_callback_with_data(self, cb, user_data):
		self.last_cb = _libwte.cruising_data_cb(cb)
		self.last_data = user_data
		_libwte.wte_set_cruising_callback_with_data(self.w, self.last_cb, self.last_data)
	

	# cruising strategy
	class cruising_strategy(Enum):
		NONE = 0
		DRIVER_RANDOM_NODE = 1
		CALLBACK = 2
		RANDOM_WALK = 3
	
	@property
	def strategy(self):
		return wait_time_estimator.cruising_strategy(_libwte.wte_get_cruising_strategy(self.w))
	
	@strategy.setter
	def strategy(self, value):
		if type(value) is not wait_time_estimator.cruising_strategy:
			raise ArgumentTypeError("Must provide a cruising strategy value!\n")
		_libwte.wte_set_cruising_strategy(self.w, value.value)
	
	def read_cruising_target_dist(self, hour, fn):
		res = _libwte.wte_read_cruising_target_dist(self.w, hour, bytes(fn, 'utf-8'))
		if res == 0:
			self.handle_error()
	
	def read_cruising_target_dist_mult(self, fn):
		res = _libwte.wte_read_cruising_target_dist_mult(self.w, bytes(fn, 'utf-8'))
		if res == 0:
			self.handle_error()
			
	def read_road_network(self, fn, is_csv = False, has_header = False):
		"""
			Read the road network used for random walk cruising.
			
			Parameters:
				fn: name of the file containing the network (as a directed edgelist)
				is_csv: indicates if the file is in CSV format (default: TSV)
				has_header: indicates if the file has a header (if yes, the first line is skipped)
		"""
		res = _libwte.wte_read_road_network(self.w, bytes(fn, 'utf-8'), 1 if is_csv else 0, 1 if has_header else 0)
		if res == 0:
			self.handle_error()
	
	@property
	def random_walk_max_dist(self):
		"""
			Maximum distance for random walk cruising.
			Drivers stay idle after reaching this distance.
		"""
		return _libwte.wte_get_random_walk_max_dist(self.w)
	
	@random_walk_max_dist.setter
	def random_walk_max_dist(self, value):
		_libwte.wte_set_random_walk_max_dist(self.w, value)
	
	@property
	def driver_max_idle_time(self):
		"""
			Maximum idle time for drivers. They are automatically removed after this time.
		"""
		return _libwte.wte_get_driver_max_idle(self.w)
	
	@driver_max_idle_time.setter
	def driver_max_idle_time(self, t):
		_libwte.wte_set_driver_max_idle(self.w, t)
		
	
	def prune_drivers(self, max_time = None):
		"""
			Go through all drivers and remove all that have been idle
			for more than the given max_time parameter (if not None),
			or the value previously set with driver_max_idle_time.
		"""
		old_time = None
		if max_time is not None:
			old_time = self.driver_max_idle_time
			self.driver_max_idle_time = max_time
		res = _libwte.wte_prune_drivers(self.w)
		if res == 0:
			self.handle_error()
		if max_time is not None:
			self.driver_max_idle_time = old_time
	
	def remove_driver(self, driver_id):
		"""
			Remove the driver with the given ID. This driver needs to be
			either idle or cruising (it is an error to call with a driver
			assigned to a trip).
		"""
		res = _libwte.wte_remove_driver(self.w, driver_id)
		if res == 0:
			self.handle_error()
		
	
	
	def filter_trips(self, filter_ids):
		filter_tmp = (c_ulong * len(filter_ids))()
		filter_tmp[:] = filter_ids
		_libwte.wte_filter_trips(self.w, filter_tmp, len(filter_tmp))
	
	def copy_trips_filter(self, filter_ids = None):
		if filter_ids is None:
			wtmp = _libwte.wte_copy_trips_filter(self.w, None, 0)
			if wtmp == 0 or wtmp is None:
				self.handle_error()
			return wait_time_estimator(c_void_p(wtmp))
		else:
			filter_tmp = (c_ulong * len(filter_ids))()
			filter_tmp[:] = filter_ids
			wtmp = _libwte.wte_copy_trips_filter(self.w, filter_tmp, len(filter_tmp))
			if wtmp == 0 or wtmp is None:
				self.handle_error()
			return wait_time_estimator(c_void_p(wtmp))
	
	@property
	def handle_exceptions(self):
		"""
			Sets if errors should be converted to Python exceptions. Default is True.
			Setting this to False will result in errors crashing the program.
			This can be useful e.g. when running from a debugger.
		"""
		return _libwte.wte_get_handle_exceptions(self.w)
	
	@handle_exceptions.setter
	def handle_exceptions(self, value):
		_libwte.wte_set_handle_exceptions(self.w, 1 if value else 0)

