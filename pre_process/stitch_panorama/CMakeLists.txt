cmake_minimum_required(VERSION 2.8)

project(stitch_panorama)
find_package( OpenCV REQUIRED )
FIND_PACKAGE(Ceres REQUIRED)
# FIND_PACKAGE(gflags REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
LINK_DIRECTORIES(/usr/local/lib)

set( CMAKE_CXX_FLAGS "-Wno-c++11-extensions -Wno-gnu-static-float-init -Wno-sign-compare" )
#set( CMAKE_CXX_FLAGS "-Wno-c++11-extensions -Wno-gnu-static-float-init -Wno-sign-compare -fopenmp" )

#set(CMAKE_C_COMPILER clang-omp CACHE STRING "C compiler" FORCE)
#set(CMAKE_CXX_COMPILER clang-omp++ CACHE STRING "C++ compiler" FORCE)

#find_package(OpenMP REQUIRED)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")



add_executable(stitch_panorama_cli stitch_panorama_cli.cc )
target_link_libraries(stitch_panorama_cli stitch_panorama)
target_link_libraries(stitch_panorama_cli ${OpenCV_LIBS} )
TARGET_LINK_LIBRARIES(stitch_panorama_cli ceres)
TARGET_LINK_LIBRARIES(stitch_panorama_cli gflags)

add_library(stitch_panorama stitch_panorama.cc stitch_panorama.h)
