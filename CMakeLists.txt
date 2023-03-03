cmake_minimum_required(VERSION 3.00)
project(vslam-pose-estimation-framework LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(modules)

add_executable(image_process examples/image_process.cpp)
target_link_libraries(image_process PUBLIC image_module)