cmake_minimum_required(VERSION 3.21)
project(RGLServerPlugin)
set(CMAKE_CXX_STANDARD 20)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

include_directories(include)

add_library(RGLServerPluginManager SHARED src/RGLServerPluginManager.cc src/Mesh.cc src/Utils.cc src/Scene.cc)
target_include_directories(RGLServerPluginManager PRIVATE RobotecGPULidar)
target_link_libraries(RGLServerPluginManager
    PRIVATE ignition-gazebo6::ignition-gazebo6
    PRIVATE ignition-plugin1::ignition-plugin1
    PRIVATE RobotecGPULidar
)
set_target_properties(RGLServerPluginManager PROPERTIES INSTALL_RPATH "$ORIGIN")

add_library(RGLServerPluginInstance SHARED src/RGLServerPluginInstance.cc src/Lidar.cc src/Utils.cc src/LidarPatternLoader.cc)
target_include_directories(RGLServerPluginInstance PRIVATE RobotecGPULidar)
target_link_libraries(RGLServerPluginInstance
    PRIVATE ignition-gazebo6::ignition-gazebo6
    PRIVATE ignition-plugin1::ignition-plugin1
    PRIVATE RobotecGPULidar
)
set_target_properties(RGLServerPluginInstance PROPERTIES INSTALL_RPATH "$ORIGIN")

## Install libraries

install(TARGETS RGLServerPluginInstance RGLServerPluginManager
    DESTINATION RGLServerPlugin
)
# RobotecGPULidar needs to be installed manually as it is a dependent shared library
install(IMPORTED_RUNTIME_ARTIFACTS RobotecGPULidar
    DESTINATION RGLServerPlugin
)
