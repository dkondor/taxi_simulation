# taxi_simulation

Simulate simple model of taxi dispatching, measure the ratio of served / rejected trips, waiting times and potentially other statistics.

Example usage is included in the Jupyter notebook. Below are a set of steps to download and prepare the necessary data. Example dataset available at https://zenodo.org/record/5594338

## 1. Download the code and compile it

This needs a C++14 compiler (tested on recent versions of [GCC](https://www.gnu.org/software/gcc/) or [clang](https://clang.llvm.org/)).

Download the code in this repository:
```
git clone https://github.com/dkondor/taxi_simulation.git
```
This will create a directory `taxi_simulation`; change to this directory (all later command are run here):
```
cd taxi_simulation
```

Compile the main library needed for running the simulation:
```
g++ -c wte_class.cpp wte_class_interface.cpp -O3 -march=native -std=gnu++14 -fPIC
g++ -shared -Wl,-soname,libwte.so -o libwte.so wte_class.o wte_class_interface.o
```

Optionally, compile the programs necessary for estimating travel times in the network (only needed if you would like to regenerate the travel time matrix; if using the one shared on [Zenodo](https://zenodo.org/record/5594338), you can skip this step):
```
g++ -o tte travel_time_estimator.cpp -O3 -march=native -std=gnu++14 -lm
g++ -o ttu travel_time_unmatched.cpp -O3 -march=native -std=gnu++14 -lm
g++ -o nsp network_shortest_path2.cpp -O3 -march=native -std=gnu++11 -lm
g++ -o csi create_sorted_index.cpp -O3 -march=native -std=gnu++14 -lm -pthread
g++ -o cpm create_path_matrix.cpp -O3 -march=native -std=gnu++14 -lm -pthread
```

## 2. Download the example data

Download and unzip the travel time matrix, either with the following commands:
```
mkdir data
cd data
wget https://zenodo.org/record/5594338/files/nyc_travel_times.zip
unzip nyc_travel_times.zip
cd ..
```
or manually from the following link:  
https://zenodo.org/record/5594338/files/nyc_travel_times.zip  
and unzip it in a `data` subdirectory.

Download the set of trips and additional necessary data using the scripts [download_trips_3weeks.sh](download_trips_3weeks.sh) (only three weeks of example data needed) or [download_trips_all.sh](download_trips_all.sh) (one year data). These can be run directly from the main directory, e.g.:
```
./download_trips_3weeks.sh
```
(note: these scripts require `wget` to do the actual downloads; alternatively, download the files manually and put them under `data/nyc_travel_times`)

As a result, the main directory should contain a `data` subdirectory with the following contents:
```
data/
	nytrips/
		nytrips_15180.bz2
		...
	nyc_travel_times/
		Ho0.bin
		...
```

### 3. Run the examples

Start [Jupyter](https://jupyter.org/install) from this directory and open the [simulation_examples_nyc.ipynb](simulation_examples_nyc.ipynb) notebook for simulation examples:
```
jupyter-notebook
```


### 4. Re-create the travel time matrices

The script [nyc_travel_times.fsh](nyc_travel_times.fsh) contains the steps necessary to create the binary matrices of travel times used by the simulation (note: running this script requires the [fish shell](https://fishshell.com/)). This is not necessary for running the above steps, but can be used as the basis for preparing other datasets.


