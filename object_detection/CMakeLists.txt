cmake_minimum_required(VERSION 2.8)
project(object_detection)

FIND_PACKAGE(OpenCV REQUIRED)

link_directories(/usr/local/lib)

if(UNIX)
set(CMAKE_CXX_FLAGS "-Wno-c++11-extensions -std=c++11")
endif(UNIX)

if(${CMAKE_SYSTEM} MATCHES "Darwin")
   set( CMAKE_CXX_FLAGS "-Wno-c++11-extensions -Wno-gnu-static-float-init -Wno-sign-compare" )
endif(${CMAKE_SYSTEM} MATCHES "Darwin")

if (WIN32)
	include_directories("C:\\Eigen3.2.2")
	include_directories("C:\\gflags-2.1.1\\include")
	link_directories("C:\\gflags-2.1.1\\lib")	
endif (WIN32)

add_executable( generate_object_icons_cli generate_object_icons_cli.cc detection.cc ../base/panorama.cc ../base/point_cloud.cc )
target_link_libraries( generate_object_icons_cli ${OpenCV_LIBS} )
target_link_libraries( generate_object_icons_cli gflags )
