cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(delaunay_triangulation-example)

add_executable(delaunay_triangulation delaunay_triangulation.cpp)
target_link_libraries(delaunay_triangulation visualization ${GMP_LIBRARIES})
