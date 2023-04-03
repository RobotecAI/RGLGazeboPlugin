cmake_minimum_required(VERSION 3.16)
project(RGLGazeboPlugin)

set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/install")

add_subdirectory(RGLServerPlugin)
add_subdirectory(RGLVisualize)
