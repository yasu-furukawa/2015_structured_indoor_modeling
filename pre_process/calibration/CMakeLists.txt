cmake_minimum_required(VERSION 2.8)
project(align_images_cli)
find_package( OpenCV REQUIRED )
FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
LINK_DIRECTORIES(/usr/local/lib)

if(UNIX)
set(CMAKE_CXX_FLAGS "-Wno-c++11-extensions -std=c++11")
endif(UNIX)

if(${CMAKE_SYSTEM} MATCHES "Darwin")
   set( CMAKE_CXX_FLAGS "-Wno-c++11-extensions -Wno-gnu-static-float-init -Wno-sign-compare" )
endif(${CMAKE_SYSTEM} MATCHES "Darwin")

set(CMAKE_CXX_FLAGS "-Wno-c++11-extensions")
add_executable( align_images_cli align_images_cli.cc align_images.cc transformation.cc )
target_link_libraries( align_images_cli ${OpenCV_LIBS} )
TARGET_LINK_LIBRARIES(align_images_cli ceres)
TARGET_LINK_LIBRARIES(align_images_cli gflags)
TARGET_LINK_LIBRARIES(align_images_cli glog)

add_executable( align_panorama_to_depth_cli align_panorama_to_depth_cli.cc transformation.cc depthmap_refiner.cc )
target_link_libraries( align_panorama_to_depth_cli ${OpenCV_LIBS} )
TARGET_LINK_LIBRARIES(align_panorama_to_depth_cli ceres)
TARGET_LINK_LIBRARIES(align_panorama_to_depth_cli gflags)
TARGET_LINK_LIBRARIES(align_panorama_to_depth_cli glog)


add_executable( render_ply_to_panorama_cli render_ply_to_panorama_cli.cc transformation.cc depthmap_refiner.cc )
target_link_libraries( render_ply_to_panorama_cli ${OpenCV_LIBS} )
TARGET_LINK_LIBRARIES( render_ply_to_panorama_cli ceres)
TARGET_LINK_LIBRARIES( render_ply_to_panorama_cli gflags)

add_executable( generate_depthmaps_cli generate_depthmaps_cli.cc ../../base/panorama.cc ../../base/point_cloud.cc )
target_link_libraries( generate_depthmaps_cli ${OpenCV_LIBS} )
TARGET_LINK_LIBRARIES( generate_depthmaps_cli ceres)
TARGET_LINK_LIBRARIES( generate_depthmaps_cli gflags)
