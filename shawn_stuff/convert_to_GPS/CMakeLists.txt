cmake_minimum_required(VERSION 2.8)

find_package (GeographicLib 1.34 REQUIRED)
include_directories (${GeographicLib_INCLUDE_DIRS})
add_definitions (${GeographicLib_DEFINITIONS})
add_executable (program source.cpp)
target_link_libraries (program ${GeographicLib_LIBRARIES})

