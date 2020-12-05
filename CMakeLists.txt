# Sets the minimum required version of CMake for a project.
cmake_minimum_required(VERSION 3.15)

# Sets the name of the project, and stores it in the variable PROJECT_NAME.
project(ExtendedKF)

# Set a variable to a given value.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(sources src/FusionEKF.cpp src/kalman_filter.cpp src/main.cpp src/tools.cpp)

find_package(Threads REQUIRED)
find_package(pubsub_client REQUIRED)

# MSVC requires some additional code to select the correct runtime library
if (VCPKG_TARGET_TRIPLET MATCHES "-static$")
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else ()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif ()

# Adds definitions to the compiler command line for targets in the current directory.
add_definitions(-std=c++0x)

# Add an executable to the project using the specified source files.
add_executable(ExtendedKF ${sources})

target_link_libraries(ExtendedKF googleapis-c++::pubsub_client)
