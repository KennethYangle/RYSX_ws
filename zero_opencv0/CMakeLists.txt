project(test)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(OpenCV 3.3.1 REQUIRED)
#FIND_PACKAGE(Boost COMPONENTS filesystem system thread REQUIRED)

include_directories("/usr/include")
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(test test.cpp)
target_link_libraries(test
        ${OpenCV_LIBS}
        )



