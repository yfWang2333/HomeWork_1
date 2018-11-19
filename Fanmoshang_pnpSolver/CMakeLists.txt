cmake_minimum_required(VERSION 3.12)
project(pnpSolver)
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /home/top/project/vision/opencv-3.2.0)
message(aaa:${CMAKE_PREFIX_PATH})
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)
file(GLOB pro_file *.cpp)
add_executable(pnpSolver ${pro_file})
#include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)
target_link_libraries(pnpSolver ${OpenCV_LIBS}) # replace binary_name