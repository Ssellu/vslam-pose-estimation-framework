cmake_minimum_required(VERSION 3.00)

project(image_module LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(IMAGE_MODULE_SOURCE_FILES src/extract_features.cpp)

add_library(image_module ${IMAGE_MODULE_SOURCE_FILES})

find_package(OpenCV REQUIRED HINTS ${CMAKE_SOURCE_DIR}/thirdparty/OpenCV/install/lib/cmake/opencv4)

if (OpenCV_FOUND)
    message(STATUS "OpenCV Found! - ${OpenCV_DIR}")
endif()

target_include_directories(image_module PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS})
target_link_libraries(image_module PUBLIC
        ${OpenCV_LIBS})
