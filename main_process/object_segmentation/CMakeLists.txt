cmake_minimum_required(VERSION 2.8)
project( object_segmentation_cli )
find_package( OpenCV REQUIRED )
#find_package( PCl 1.7 REQUIRED COMPONENTS common io)
#find_package( PCl 1.7 REQUIRED COMPONENTS common)

# FIND_PACKAGE( Ceres REQUIRED )
# INCLUDE_DIRECTORIES( ${CERES_INCLUDE_DIRS} )

#include_directories(${PCL_INCLUDE_DIRS})


#link_directories(${PCL_LIBRARY_DIRS})
LINK_DIRECTORIES( /usr/local/lib )

#add_definitions(${PCL_DEFINITIONS})

if(${CMAKE_SYSTEM} MATCHES "Linux")
  include_directories("/usr/include/eigen3")
  set(CMAKE_CXX_FLAGS "-Wno-c++11-extensions -std=c++11")
endif(${CMAKE_SYSTEM} MATCHES "Linux")

if(${CMAKE_SYSTEM} MATCHES "Darwin")
   include_directories("/usr/local/include/eigen3")
   set( CMAKE_CXX_FLAGS "-Wno-c++11-extensions -Wno-gnu-static-float-init -Wno-sign-compare" )
endif(${CMAKE_SYSTEM} MATCHES "Darwin")

add_executable( object_segmentation_cli object_segmentation_cli.cc object_segmentation.cc ../../base/floorplan.cc ../../base/indoor_polygon.cc ../../base/point_cloud.cc ../../base/kdtree/KDtree.cc )

target_link_libraries( object_segmentation_cli ${OpenCV_LIBS} )
target_link_libraries( object_segmentation_cli gflags )

if(${CMAKE_SYSTEM} MATCHES "Linux")
  target_link_libraries(object_segmentation_cli pthread)
endif(${CMAKE_SYSTEM} MATCHES "Linux")

#target_link_libraries( object_segmentation_cli ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})


# target_link_libraries( object_segmentation_cli ceres )
# target_link_libraries( object_segmentation_cli glog )
