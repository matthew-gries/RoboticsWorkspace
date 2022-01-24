cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)
project(OpenCVExample)

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV Includes ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV Libraries ${OpenCV_LIBS}")

include_directories(${OpenCV_INCLUDE_DIRS})

file(
    GLOB_RECURSE
    SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
)

add_executable(OpenCVExample ${SOURCE_FILES})
target_link_libraries(OpenCVExample ${OpenCV_LIBS})