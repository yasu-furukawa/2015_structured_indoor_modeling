cmake_minimum_required(VERSION 2.8)
project(generate_texture_cli)

FIND_PACKAGE(OpenCV REQUIRED)

#find_package( OpenCV REQUIRED )
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

add_executable( generate_texture_floorplan_cli generate_texture.cc generate_texture_floorplan_cli.cc generate_texture_floorplan.cc synthesize.cc ../base/floorplan.cc ../base/geometry.cc ../base/panorama.cc ../base/point_cloud.cc ../base/imageProcess/morphological_operation.cc )
target_link_libraries( generate_texture_floorplan_cli ${OpenCV_LIBS} )
TARGET_LINK_LIBRARIES( generate_texture_floorplan_cli gflags )

add_executable( generate_texture_indoor_polygon_cli generate_texture.cc generate_texture_indoor_polygon_cli.cc generate_texture_indoor_polygon.cc synthesize.cc ../base/indoor_polygon.cc ../base/geometry.cc ../base/panorama.cc ../base/point_cloud.cc ../base/imageProcess/morphological_operation.cc )
target_link_libraries( generate_texture_indoor_polygon_cli ${OpenCV_LIBS} )
TARGET_LINK_LIBRARIES( generate_texture_indoor_polygon_cli gflags )

add_executable( color_point_cloud_cli color_point_cloud_cli.cc generate_texture.cc synthesize.cc ../base/floorplan.cc ../base/geometry.cc ../base/panorama.cc ../base/point_cloud.cc ../base/imageProcess/morphological_operation.cc ../base/kdtree/KDtree.cc )
target_link_libraries( color_point_cloud_cli ${OpenCV_LIBS} )
TARGET_LINK_LIBRARIES( color_point_cloud_cli gflags )

add_executable( generate_thumbnail_cli generate_thumbnail_cli.cc ../base/floorplan.cc ../base/geometry.cc ../base/panorama.cc )
target_link_libraries( generate_thumbnail_cli ${OpenCV_LIBS} )
target_link_libraries( generate_thumbnail_cli gflags )
