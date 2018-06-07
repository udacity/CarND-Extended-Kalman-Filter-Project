echo "Compiling main.cpp"
g++ -o build/ExtendedKF.out src/main.cpp src/FusionEKF.cpp src/kalman_filter.cpp src/tools.cpp -std=c++11 /usr/lib/libuWS.so /usr/lib/x86_64-linux-gnu/libssl.so /usr/lib/x86_64-linux-gnu/libz.so

