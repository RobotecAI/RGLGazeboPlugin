cmake_minimum_required(VERSION 3.21)
project(RGLGazeboPlugin)

set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/install")

add_subdirectory(external)
add_subdirectory(RGLServerPlugin)
add_subdirectory(RGLVisualize)
