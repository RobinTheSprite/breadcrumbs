cmake_minimum_required(VERSION 3.10)
project(pathfinder)

set(CMAKE_CXX_STANDARD 17)

find_package(PDAL 1.9.0 REQUIRED CONFIG)
include_directories(${PDAL_INCLUDE_DIRS})
link_directories(${PDAL_LIBRARY_DIRS})
add_definitions(${PDAL_DEFINITIONS})

#add_library(pathfinder SHARED MyReader.cpp)
add_executable(pathfinder main.cpp)
target_include_directories(pathfinder PRIVATE
        ${PDAL_INCLUDE_DIRS})
target_link_libraries(pathfinder PRIVATE ${PDAL_LIBRARIES} ${LASZIP_LIBRARY})