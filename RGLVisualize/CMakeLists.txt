cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

# This is required for Qt
if(POLICY CMP0100)
    cmake_policy(SET CMP0100 NEW)
endif()

project(RGLVisualize)

set(CMAKE_AUTOMOC ON)

find_package(gz-cmake3 REQUIRED)
gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

gz_find_package(gz-rendering8 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})


gz_find_package(gz-gui8 REQUIRED)
set(GZ_GUI_VER ${gz-gui8_VERSION_MAJOR})

QT5_ADD_RESOURCES(resources_RCC RGLVisualize.qrc)

add_library(RGLVisualize SHARED
    RGLVisualize.cc
    ${resources_RCC}
)

target_link_libraries(RGLVisualize
    PRIVATE
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
    gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
)

## Install libraries

install(TARGETS RGLVisualize
    DESTINATION RGLVisualize
)
