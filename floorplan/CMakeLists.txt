# Created by the script cgal_create_cmake_script_with_options
# This is the CMake script for compiling a set of CGAL applications.

project( floorplan_generation_cli )

find_package( OpenCV REQUIRED )

cmake_minimum_required(VERSION 2.6.2)
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" VERSION_GREATER 2.6)
  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" VERSION_GREATER 2.8.3)
    cmake_policy(VERSION 2.8.4)
  else()
    cmake_policy(VERSION 2.6)
  endif()
endif()

set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true )
 
if ( COMMAND cmake_policy )

  cmake_policy( SET CMP0003 NEW )  

endif()

# CGAL and its components
find_package( CGAL QUIET COMPONENTS Core )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()

# include helper file
include( ${CGAL_USE_FILE} )


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()

set(CMAKE_CXX_FLAGS "-Wno-c++11-extensions")

# include for local directory

# include for local package


# Creating entries for target: floorplan_generation_cli
# ############################

add_executable( floorplan_generation_cli floorplan_generation_cli.cc floorplan.cc floorplan_util.cc polygon_triangulation2.cc)
add_to_cached_list( CGAL_EXECUTABLE_TARGETS floorplan_generation_cli )
target_link_libraries(floorplan_generation_cli   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )


add_executable( generate_thumbnail_cli generate_thumbnail_cli.cc floorplan.cc ../viewer/configuration.cc )
target_link_libraries( generate_thumbnail_cli ${OpenCV_LIBS} )
target_link_libraries( generate_thumbnail_cli gflags )
