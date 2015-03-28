cmake_minimum_required(VERSION 2.8)
project(Object_refinement)

find_package(OpenCV REQUIRED)

if(${CMAKE_SYSTEM} MATCHES "Linux")
find_package(gflags REQUIRED)
endif(${CMAKE_SYSTEM} MATCHES "Linux")

set( CMAKE_CXX_FLAGS "-std=c++11 -Wno-c++11-extensions -Wno-gnu-static-float-init -Wno-sign-compare -DUSE_64_BIT_PTR_CAST" )

if(${CMAKE_SYSTEM} MATCHES "Darwin")
	include_directories(/usr/local/include/eigen3)
	link_directories(/usr/local/lib)
elseif(${CMAKE_SYSTEM} MATCHES "Linux")
	include_directories("/usr/local/include")
	include_directories("/usr/include/eigen3")
endif(${CMAKE_SYSTEM} MATCHES "Darwin")

add_executable(Object_refinement object_refinement.cpp SLIC/SLIC.cpp object_refinement_cali.cpp depth_filling.cpp ../../base/point_cloud.cc ../../base/panorama.cc MRF/BP-S.cpp MRF/GCoptimization.cpp MRF/ICM.cpp MRF/LinkedBlockList.cpp MRF/MaxProdBP.cpp MRF/TRW-S.cpp MRF/graph.cpp MRF/maxflow.cpp MRF/mrf.cpp MRF/regions-maxprod.cpp)

target_link_libraries(Object_refinement gflags)
target_link_libraries(Object_refinement opencv_core opencv_highgui opencv_imgproc opencv_flann)
# target_link_libraries(Object_hole_filling MRF)

if(${CMAKE_SYSTEM} MATCHES "Linux")
	target_link_libraries(Object_refinement pthread)
endif(${CMAKE_SYSTEM} MATCHES "Linux")
