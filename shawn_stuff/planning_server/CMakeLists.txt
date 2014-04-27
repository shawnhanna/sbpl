cmake_minimum_required(VERSION 2.8)
project(test_sbpl)

set(CMAKE_BUILD_TYPE Release)

find_package(Boost)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

find_package (GeographicLib 1.34 REQUIRED)
include_directories (${GeographicLib_INCLUDE_DIRS})
add_definitions (${GeographicLib_DEFINITIONS})


add_executable(planner_server main.cpp Planner.cpp)
target_link_libraries(planner_server ${SBPL_LIBRARIES} ${BOOST_LIBRARIES} ${GeographicLib_LIBRARIES} pthread boost_system)
