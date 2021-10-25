#!/usr/bin/fish
# Script to create the travel time matrices used by the simulation.
# Note: this script requires the Fish shell (https://fishshell.com/).
# It is assumed to be run in the main directory of the git repository.
# Adjust the variables code_dir and trips_dir and out_dir below to use
# other directories.


# 1. Set the input and output directories.
# 1.1. location of the programs used (assumed to be the current directory)
set code_dir .
# 1.2. location of the taxi trips
set trips_dir data/nytrips
# 1.3. output directory
set out_dir data/nyc_travel_times
# create the output directory
mkdir -p $out_dir


# 2. Generate travel times on edges with data

# 2.1. Separate trips by hour of day for faster processing later
# (note: mawk is used here for faster speed; you can replace it with awk, or gawk if it is not available on your system)
bzcat $trips_dir/nytrips_*.bz2 | mawk -F, -v out_base=$out_dir/trip_hour_ '{h1 = int(($2 % 86400) / 3600); fn = out_base""h1".csv"; print $0 >> fn;}'

# 2.2. Run the edge travel time calculation for each hour
for h in (seq 0 23)
$code_dir/tte -n $trips_dir/NYC_segments.csv -d 0 5 < $out_dir/trip_hour_$h.csv > $out_dir/res_Week_h$h.out
echo $h
end

# 2.3. Remove temporary files created before
rm $out_dir/trip_hour_*


# 3. Generate travel times on the remaining edges
for h in (seq 0 23)
$code_dir/ttu -n $trips_dir/NYC_segments.csv < $out_dir/res_Week_h$h.out > $out_dir/res_unmatched_Week_h$h.out
end


# 4. Create the binary travel time matrix based on shortest paths
for h in (seq 0 23)
$code_dir/nsp -1 -c -n $out_dir/res_unmatched_Week_h$h.out -o $out_dir/Ho$h.bin
end

# 5. Remove temporary files
rm $out_dir/*.out

# 6. Create index matrices for travel times
$code_dir/csi -M 4092 -h 0 24 -b $out_dir/Ho -o $out_dir/index

# 7. Create a binary travel distance matrix (optional)
$code_dir/nsp -c -o $out_dir/distances_dir.bin < $trips_dir/NYC_segments.csv

# 8. Optional: create matrices storing the shortest time paths (not used by the simulation currently)
$code_dir/cpm -t $out_dir/Ho -o $out_dir/paths -n $trips_dir/NYC_segments.csv -c -m 4092




