cmake_minimum_required(VERSION 3.5)
project(libcalib)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_FLAGS "-std=c++14 -w -msse4.1 -fopenmp")
set(CMAKE_EXE_LINKER_FLAGS "-fPIE -pie")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -s")
set(CMAKE_SKIP_BUILD_RPATH OFF)
set(CMAKE_DEBUG_POSTFIX _d)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(SHARED_LIB "Request build of shared libraries." ON)
if (SHARED_LIB)
  set(LIB_TYPE SHARED)
else ()
  set(LIB_TYPE STATIC)
endif ()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

add_subdirectory(${PROJECT_SOURCE_DIR}/sample)
add_subdirectory(${PROJECT_SOURCE_DIR}/src)

