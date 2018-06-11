#! /bin/bash

# clean
cd /home/workspace/CarND-Extended-Kalman-Filter-Project
rm -rf build

# build
mkdir build

# compile based on environment {local || workspace}
if [[ -v WORKSPACEID ]]; then
	echo compiling workspace main
    ./make.sh
    cd build
    # run workspace version
    ./ExtendedKF.out
else 
	echo  compiling local main
    cd build
    cmake .. && make
    # run local version
    ./ExtendedKF    
fi