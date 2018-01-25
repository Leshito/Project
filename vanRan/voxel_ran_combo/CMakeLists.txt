cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(voxel_ran_combo)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB SOURCES src/*.cpp)


add_executable (rewtMain ${SOURCES})

#target_link_libraries (random_sample_consensus ${PCL_LIBRARIES})
#target_link_libraries (voxel_grid ${PCL_LIBRARIES})
target_link_libraries (rewtMain ${PCL_LIBRARIES})
