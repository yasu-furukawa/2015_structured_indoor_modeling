cmake_minimum_required(VERSION 2.8)
project(Object_hole_filling)

set( CMAKE_CXX_FLAGS "-std=c++11 -Wno-c++11-extensions -Wno-gnu-static-float-init -Wno-sign-compare" )

if(UNIX)
	link_directories("/usr/lib/x86_64-linux-gnu")
endif(UNIX)

find_package(gflags REQUIRED)
find_package(OpenCV REQUIRED)
include_directories($(EIGEN_INCLUDE_DIR))


add_executable(Object_hole_filling object_hole_filling.cpp SLIC.cpp object_hole_filling_cali.cpp ../base/point_cloud.cc)

target_link_libraries(Object_hole_filling gflags)
target_link_libraries(Object_hole_filling opencv_core opencv_highgui opencv_imgproc)

if(UNIX)
	target_link_libraries(Object_hole_filling pthread)
endif(UNIX)
