cmake_minimum_required(VERSION 3.18)
project(RGLServerPlugin)
set(CMAKE_CXX_STANDARD 20)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

include_directories(include)

include_directories(/home/jakub/CLionProjects/RobotecGPULidar/include)

add_library(RGLServerPluginManager SHARED src/RGLServerPluginManager.cc src/Mesh.cc src/Utils.cc src/Scene.cc)
target_link_libraries(RGLServerPluginManager
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::ignition-plugin1
    "/home/jakub/CLionProjects/RobotecGPULidar/build/libRobotecGPULidar.so"
)

add_library(RGLServerPluginInstance SHARED src/RGLServerPluginInstance.cc src/Lidar.cc src/Utils.cc)
target_link_libraries(RGLServerPluginInstance
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::ignition-plugin1
    "/home/jakub/CLionProjects/RobotecGPULidar/build/libRobotecGPULidar.so"
)
