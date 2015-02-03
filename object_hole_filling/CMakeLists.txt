cmake_minimum_required(VERSION 2.8)
project(Object_hole_filling)

find_package(gflags REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(Object_hole_filling object_hole_filling.cpp SLIC.cpp object_hole_filling_cali.cpp)

target_link_libraries(Object_hole_filling gflags)
target_link_libraries(Object_hole_filling opencv_core opencv_highgui opencv_imgproc)

