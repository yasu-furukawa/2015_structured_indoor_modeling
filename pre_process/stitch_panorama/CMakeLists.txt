cmake_minimum_required(VERSION 2.8)

project(stitch_panorama)
find_package( OpenCV REQUIRED )
FIND_PACKAGE(Ceres REQUIRED)
# FIND_PACKAGE(gflags REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
LINK_DIRECTORIES(/usr/local/lib)

if(${CMAKE_SYSTEM} MATCHES "Darwin")
   set( CMAKE_CXX_FLAGS "-Wno-c++11-extensions -Wno-gnu-static-float-init -Wno-sign-compare" )
endif(${CMAKE_SYSTEM} MATCHES "Darwin")

set(CMAKE_CXX_FLAGS "-Wno-c++11-extensions")

add_executable(stitch_panorama_cli stitch_panorama_cli.cc )
target_link_libraries(stitch_panorama_cli stitch_panorama)
target_link_libraries(stitch_panorama_cli ${OpenCV_LIBS} )
TARGET_LINK_LIBRARIES(stitch_panorama_cli ceres)
TARGET_LINK_LIBRARIES(stitch_panorama_cli gflags)

add_library(stitch_panorama stitch_panorama.cc stitch_panorama.h)
