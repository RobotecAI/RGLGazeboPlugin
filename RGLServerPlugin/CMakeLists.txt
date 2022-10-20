cmake_minimum_required(VERSION 3.18)
project(RGLServerPlugin)
set(CMAKE_CXX_STANDARD 20)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

# External dependencies
add_subdirectory(external)

include_directories(include)

add_library(RGLServerPlugin SHARED src/RGLServerPlugin.cc src/Mesh.cc src/Utils.cc src/Lidar.cc)
target_link_libraries(RGLServerPlugin
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::ignition-plugin1
    RobotecGPULidar
)
