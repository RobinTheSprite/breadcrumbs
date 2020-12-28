cmake_minimum_required(VERSION 3.10)
project(breadcrumbs)

set(CMAKE_CXX_STANDARD 17)

find_package(TIFF REQUIRED)

add_executable(breadcrumbs main.cpp TiffOps.cpp breadcrumbs.cpp)
target_link_libraries(breadcrumbs ${TIFF_LIBRARIES})
