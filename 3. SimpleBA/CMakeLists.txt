cmake_minimum_required(VERSION 3.5)
cmake_policy(VERSION 3.5)

project(HelloCeres)

set(DEFAULT_CXX_STANDARD 14)

find_package(Eigen3 3.3 REQUIRED)
find_package(LAPACK QUIET)
# find_package(SuiteSparse)

find_package(gflags 2.2.0)
# find_package(Glog)

find_package(Ceres)

include_directories(
	include
)

add_executable(main main.cpp)
target_link_libraries(main Ceres::ceres)


