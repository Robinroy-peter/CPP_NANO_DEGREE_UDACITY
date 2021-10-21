cmake_minimum_required(VERSION 3.5.1)

project(coop)


find_package(Boost COMPONENTS thread REQUIRED)

find_package(OpenCV REQUIRED)

add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "-fpermissive")


include_directories(include ${Boost_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR})
link_directories(/usr/local/lib/)
set(CMAKE_BINARY_DIR ../bin)

add_executable(visualizer src/visualizer.cpp)

target_link_libraries(visualizer ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

