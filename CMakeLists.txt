cmake_minimum_required(VERSION 3.27.0)
project(lqr_tracking)

set(CMAKE_CXX_STANDARD 20)

set(OpenCV_DIR ${PROJECT_SOURCE_DIR}/include/opencv/install)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(./include)
include_directories(./include/eigen-3.4.0)

add_executable(lqr_tracking lqr.cpp)

target_link_libraries(lqr_tracking ${OpenCV_LIBS})
