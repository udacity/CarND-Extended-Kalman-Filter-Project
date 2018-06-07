#! /bin/bash

sudo apt-get install git libuv1-dev libssl-dev gcc g++ cmake make

cd /home/workspace/uWebSockets/
sudo make install
cd ..
apt-get update && apt-get install -y libssl-dev libuv1-dev 