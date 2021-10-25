#!/bin/sh

# script to download trip data from Zenodo
# only the three weeks used in the main simulation

# This script is supposed to be run from the main folder of the Git
# repository and will create the subdirectory data/nytrips to
# store the trips to
mkdir -p data/nytrips
cd data/nytrips

# 1st week
for d in `seq 14975 15339`
do
wget https://zenodo.org/record/5594338/files/nytrips_$d.bz2
done

# download road network and example start node distribution
wget https://zenodo.org/record/5594338/files/NYC_nodes.csv
wget https://zenodo.org/record/5594338/files/NYC_segments.csv
wget https://zenodo.org/record/5594338/files/trip_start_dist.dat
