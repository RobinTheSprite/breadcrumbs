cmake_minimum_required(VERSION 3.10)
project(breadcrumbs)

set(CMAKE_CXX_STANDARD 17)

find_package(PDAL REQUIRED CONFIG)
include_directories(${PDAL_INCLUDE_DIRS})
link_directories(${PDAL_LIBRARY_DIRS})
add_definitions(${PDAL_DEFINITIONS})

find_package(TIFF REQUIRED)

add_executable(breadcrumbs main.cpp TiffOps.cpp breadcrumbs.cpp)
target_link_libraries(breadcrumbs ${TIFF_LIBRARIES})
