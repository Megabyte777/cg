cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(minkowski_sum-example)

add_executable(minkowski_sum minkowski_sum.cpp)
target_link_libraries(minkowski_sum visualization ${GMP_LIBRARIES})
