cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

# This is required for Qt
if(POLICY CMP0100)
    cmake_policy(SET CMP0100 NEW)
endif()

project(RGLVisualize)

find_package(ignition-rendering6 REQUIRED)

set(CMAKE_AUTOMOC ON)

find_package(ignition-gui6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

QT5_ADD_RESOURCES(resources_RCC RGLVisualize.qrc)

add_library(RGLVisualize SHARED
    RGLVisualize.cc
    ${resources_RCC}
)

target_link_libraries(RGLVisualize
    PRIVATE
    ignition-gui6::ignition-gui6
    ignition-rendering6::ignition-rendering6
    ignition-msgs8::ignition-msgs8
)

## Install libraries

install(TARGETS RGLVisualize
    DESTINATION RGLVisualize
)
