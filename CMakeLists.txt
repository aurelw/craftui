cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(craftui)

# compiler options
add_definitions("-std=c++11")

# options
option (BUILD_APP "Build with application frontend." ON)
option (BUILD_TESTS "Build with tests." ON)

# deal with pcl
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# deal with boost
find_package(Boost COMPONENTS signals filesystem serialization REQUIRED)
include_directories(${Boost_INCLUDE_DIR} )

# OpenCV
find_package( OpenCV 2.3 REQUIRED )

# Protobuf
find_package(Protobuf REQUIRED)

# ZMQ
set (CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/)
find_package(ZeroMQ REQUIRED)
include_directories(${ZeroMQ_INCLUDE_DIR} )

# ImageMagick
#find_package(ImageMagick COMPONENTS Magick++)
#include_directories(${ImageMagick_INCLUDE_DIRS} )

### configure ###
set (CRAFTUI_VERSION 0.1)
set (CRAFTUI_DEBUG_LVL 4)

configure_file (
    "${PROJECT_SOURCE_DIR}/src/common/craftui_config.h.in"
    "${PROJECT_SOURCE_DIR}/src/common/craftui_config.h"
)
#################

# specify output directories
make_directory ("${CMAKE_BINARY_DIR}/bin/")
make_directory ("${CMAKE_BINARY_DIR}/bin/tests/")
make_directory ("${CMAKE_BINARY_DIR}/bin/tools/")
set (EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin") 

add_subdirectory(src)

