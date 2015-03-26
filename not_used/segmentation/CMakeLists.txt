cmake_minimum_required(VERSION 2.8)
project( segmentation_cli )
find_package( OpenCV REQUIRED )
# FIND_PACKAGE( Ceres REQUIRED )
# INCLUDE_DIRECTORIES( ${CERES_INCLUDE_DIRS} )

LINK_DIRECTORIES( /usr/local/lib )

if(UNIX)
set(CMAKE_CXX_FLAGS "-Wno-c++11-extensions -std=c++11")
endif(UNIX)

if(${CMAKE_SYSTEM} MATCHES "Darwin")
   set( CMAKE_CXX_FLAGS "-Wno-c++11-extensions -Wno-gnu-static-float-init -Wno-sign-compare" )
endif(${CMAKE_SYSTEM} MATCHES "Darwin")


add_executable( segmentation_cli segmentation_cli.cc data.cc segmentation.cc evidence.cc )
target_link_libraries( segmentation_cli ${OpenCV_LIBS} )

# target_link_libraries( segmentation_cli ceres )
# target_link_libraries( segmentation_cli gflags )
# target_link_libraries( segmentation_cli glog )
