cmake_minimum_required(VERSION 3.18)
project(RGLGazeboPlugin)
set(CMAKE_CXX_STANDARD 20)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

# External dependencies
add_subdirectory(external)

include_directories(include)

add_library(RGLServerPlugin SHARED src/server/RGLServerPlugin.cc src/server/Mesh.cc src/server/Utils.cc src/server/Lidar.cc)
target_link_libraries(RGLServerPlugin
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::ignition-plugin1
    RobotecGPULidar
)