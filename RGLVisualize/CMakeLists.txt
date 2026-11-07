cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

# This is required for Qt
if(POLICY CMP0100)
    cmake_policy(SET CMP0100 NEW)
endif()

project(RGLVisualize)

set(CMAKE_AUTOMOC ON)

find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-rendering REQUIRED)
find_package(gz_gui_vendor REQUIRED)
find_package(gz-gui REQUIRED)

QT5_ADD_RESOURCES(resources_RCC RGLVisualize.qrc)

add_library(RGLVisualize SHARED
    RGLVisualize.cc
    ${resources_RCC}
)

target_link_libraries(RGLVisualize
    PRIVATE
    gz-msgs::gz-msgs
    gz-rendering::gz-rendering
    gz-gui::gz-gui
)

## Install libraries

install(TARGETS RGLVisualize
    DESTINATION RGLVisualize
)
