cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

# This is required for Qt
if(POLICY CMP0100)
    cmake_policy(SET CMP0100 NEW)
endif()

project(RGLVisualize)

find_package(gz-rendering7 REQUIRED)

set(CMAKE_AUTOMOC ON)

find_package(gz-gui7 REQUIRED)
find_package(gz-msgs9 REQUIRED)

QT5_ADD_RESOURCES(resources_RCC RGLVisualize.qrc)

add_library(RGLVisualize SHARED
    RGLVisualize.cc
    ${resources_RCC}
)

target_link_libraries(RGLVisualize
    PRIVATE
    gz-gui7::gz-gui7
    gz-rendering7::gz-rendering7
    gz-msgs9::gz-msgs9
    )

## Install libraries

install(TARGETS RGLVisualize
    DESTINATION RGLVisualize
)
