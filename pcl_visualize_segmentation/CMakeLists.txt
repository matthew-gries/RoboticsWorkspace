cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)
project(PclVisualizeSegmentation)

find_package(PCL 1.3 REQUIRED COMPONENTS common io)

message(STATUS "PCL Includes ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL Library Dirs ${PCL_LIBRARY_DIRS}")
message(STATUS "PCL Libraries ${PCL_LIBRARIES}")
message(STATUS "PCL Definitions ${PCL_DEFINITIONS}")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(
    GLOB_RECURSE
    SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
)

add_executable(PclVisualizeSegmentation ${SOURCE_FILES})
target_link_libraries(PclVisualizeSegmentation ${PCL_LIBRARIES} pcl_segmentation pcl_visualization)