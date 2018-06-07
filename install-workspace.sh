#! /bin/bash

sudo apt-get install git libuv1-dev libssl-dev gcc g++ cmake make
cd /home/workspace/CarND-Extended-Kalman-Filter-Project
rm -rf build
mkdir build
cp make.sh build

cd /home/workspace/uWebSockets/
sudo make install
cd ..
apt-get update && apt-get install -y libssl-dev libuv1-dev 