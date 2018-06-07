#! /bin/bash

cd /home/workspace/CarND-Extended-Kalman-Filter-Project
rm -rf build
mkdir build
./make.sh
cd build
./ExtendedKF.out