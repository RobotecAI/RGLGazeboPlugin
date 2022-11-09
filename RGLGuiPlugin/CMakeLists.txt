cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

# This is required for Qt
if(POLICY CMP0100)
    cmake_policy(SET CMP0100 NEW)
endif()

project(RGLGuiPlugin)

find_package(ignition-rendering6 REQUIRED)

# GUI plugin
set(GUI_PLUGIN RGLGuiPlugin)

set(CMAKE_AUTOMOC ON)

find_package(ignition-gui6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

QT5_ADD_RESOURCES(resources_RCC ${GUI_PLUGIN}.qrc)

add_library(${GUI_PLUGIN} SHARED
        ${GUI_PLUGIN}.cc
        ${resources_RCC}
        )
target_link_libraries(${GUI_PLUGIN}
        PRIVATE
        ignition-gui6::ignition-gui6
        ignition-rendering6::ignition-rendering6
        ignition-msgs8::ignition-msgs8
        )