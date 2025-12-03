cmake_minimum_required(VERSION 3.5)
project(iitrbd-tests)

set(EIGEN_ROOT $ENV{EIGEN_ROOT}   CACHE PATH "Path to Eigen headers")

add_compile_definitions(EIGEN_NO_DEBUG)
add_compile_options(-std=c++11 -Wall -O3)

# Include directories
include_directories(${EIGEN_ROOT})


# Optional CppAD
#
option(CPPAD "Enable tests involving CppAD")
if(CPPAD)
    add_definitions(-DIITRBD_CPPAD_IS_THERE)
    set(CPPAD_ROOT $ENV{CPPAD_ROOT}   CACHE PATH "Path to CppAD headers")
    include_directories(${CPPAD_ROOT})

    add_executable(test-cppad test-cppad.cpp)
endif(CPPAD)


# Add the binaries
add_executable(basic-traits basic-traits.cpp)
add_executable(scalar-type-deduction scalar-type-deduction.cpp)
