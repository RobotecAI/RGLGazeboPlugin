cmake_minimum_required(VERSION 3.10.2)
project(RGLGazeboPlugin)
set(CMAKE_CXX_STANDARD 20)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

include_directories(include)
add_subdirectory(RobotecGPULidar)

add_library(RGLGazeboPlugin SHARED src/RGLGazeboPlugin.cc)
target_link_libraries(RGLGazeboPlugin
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::ignition-plugin1
    RobotecGPULidar
)