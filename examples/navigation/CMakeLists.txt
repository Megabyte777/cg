cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(navigation-example)

add_executable(navigation navigation.cpp)
target_link_libraries(navigation visualization ${GMP_LIBRARIES})
