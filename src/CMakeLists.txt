project(Extended_Kalman_Filter)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    ./FusionEKF.cpp
    ./kalman_filter.cpp
    ./main.cpp
    ./tools.cpp)

add_executable(ExtendedKF ${sources})
