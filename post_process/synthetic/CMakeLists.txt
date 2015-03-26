cmake_minimum_required(VERSION 2.8)
project(generate_synthetic_data_cli)

FIND_PACKAGE(OpenCV REQUIRED)

LINK_DIRECTORIES(/usr/local/lib)

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

add_executable( generate_synthetic_data_cli generate_synthetic_data_cli.cc )
TARGET_LINK_LIBRARIES( generate_synthetic_data_cli ${OpenCV_LIBS} )
TARGET_LINK_LIBRARIES( generate_synthetic_data_cli gflags )

