# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.1...3.23)

project(optimal_polygon)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# CGAL and its components
find_package(CGAL QUIET COMPONENTS)
set(CMAKE_BUILD_TYPE "Release")

if(NOT CGAL_FOUND)
  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()
endif()

# Boost and its components
find_package(Boost REQUIRED)

if(NOT Boost_FOUND)
  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()
endif()

# include for local directory

# include for local package
include_directories(BEFORE ../include)

# Creating entries for target: optimal_polygon
# ############################
add_executable(optimal_polygon src/simple_polygonization/ch2polyline_algo.cpp src/graham_scan.cpp src/simple_polygonization/incremental_algo.cpp src/optimal_polygonization/simulated_annealing_algo.cpp src/optimal_polygonization/local_transition.cpp src/optimal_polygonization/global_transition.cpp src/optimal_polygonization/subdivision.cpp src/main.cpp src/simple_polygonization/onion_algo.cpp src/optimal_polygonization/local_search_algo.cpp src/pick_algo.cpp src/utils.cpp src/optimal_polygonization/ant_colony_algo.cpp)

add_to_cached_list(CGAL_EXECUTABLE_TARGETS optimal_polygon)

# Link the executable to CGAL and third-party libraries
target_link_libraries(optimal_polygon PRIVATE CGAL::CGAL)
