cmake_minimum_required(VERSION 3.18)
project(RGLServerPlugin)
set(CMAKE_CXX_STANDARD 20)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

include_directories(include)

set(RobotecGPULidar "${CMAKE_CURRENT_SOURCE_DIR}/include/rgl/libRobotecGPULidar.so")

add_library(RGLServerPluginManager SHARED src/RGLServerPluginManager.cc src/Mesh.cc src/Utils.cc src/Scene.cc include/macros/visibility.h)
target_link_libraries(RGLServerPluginManager
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::ignition-plugin1
    ${RobotecGPULidar}
)

add_library(RGLServerPluginInstance SHARED src/RGLServerPluginInstance.cc src/Lidar.cc src/Utils.cc include/macros/visibility.h)
target_link_libraries(RGLServerPluginInstance
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::ignition-plugin1
    ${RobotecGPULidar}
)
