cmake_minimum_required(VERSION 2.8)
project(iitrbd-tests)

set(EIGEN_ROOT $ENV{EIGEN_ROOT}   CACHE PATH "Path to Eigen headers")
set(CMAKE_CXX_FLAGS "-g -Wall -O3 -D EIGEN_NO_DEBUG -std=c++11")

# Include directories
include_directories(${EIGEN_ROOT})


# Optional CppAD
#
set(CPPAD off CACHE BOOL "Enable tests of CppAD traits")
if(CPPAD)
    add_definitions(-DIITRBD_CPPAD_IS_THERE)
    set(CPPAD_ROOT $ENV{CPPAD_ROOT}   CACHE PATH "Path to CppAD headers")
    include_directories(${CPPAD_ROOT})

    add_executable(test-cppad test-cppad.cpp)
endif(CPPAD)


# Add the binaries
add_executable(basic-traits basic-traits.cpp)

