cmake_minimum_required(VERSION 2.8)
project(Object_hole_filling)

set( CMAKE_CXX_FLAGS "-std=c++11 -Wno-c++11-extensions -Wno-gnu-static-float-init -Wno-sign-compare" )

if(${CMAKE_SYSTEM} MATCHES "Darwin")
	include_directories(/usr/local/include/eigen3)
elseif(${CMAKE_SYSTEM} MATCHES "Linux")
	include_directories(/usr/local/include)
	link_directories("/usr/lib/x86_64-linux-gnu")
endif(${CMAKE_SYSTEM} MATCHES "Darwin")

find_package(gflags REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(Object_hole_filling object_hole_filling.cpp SLIC.cpp object_hole_filling_cali.cpp depth_filling.cpp ../base/point_cloud.cc ../base/panorama.cc)

target_link_libraries(Object_hole_filling gflags)
target_link_libraries(Object_hole_filling opencv_core opencv_highgui opencv_imgproc)

if(${CMAKE_SYSTEM} MATCHES "Linux")
	target_link_libraries(Object_hole_filling pthread)
endif(${CMAKE_SYSTEM} MATCHES "Linux")
