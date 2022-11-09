cmake_minimum_required(VERSION 3.18)
project(RGLServerPlugin)
set(CMAKE_CXX_STANDARD 20)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

include_directories(include)

set(RGL_VERSION "0.11.0")
set(RGL_ALPHA_VERSION "1")
set(RGL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include/rgl")
set(RGL_API_PATH "${RGL_PATH}/api")
set(RGL_SO_URL "https://github.com/RobotecAI/RobotecGPULidar/releases/download/v${RGL_VERSION}-alpha-${RGL_ALPHA_VERSION}/libRobotecGPULidar.so")
set(RGL_API_URL "https://raw.githubusercontent.com/RobotecAI/RobotecGPULidar/v11/include/rgl/api/core.h")

# you have to delete old files to download new ones if a new version is available
if(NOT EXISTS "${RGL_PATH}/libRobotecGPULidar.so")
    execute_process(COMMAND wget ${RGL_SO_URL} -P ${RGL_PATH} -q)
endif()

if(NOT EXISTS "${RGL_API_PATH}/core.h")
    execute_process(COMMAND wget ${RGL_API_URL} -P ${RGL_API_PATH} -q)
endif()

set(RobotecGPULidar "${RGL_PATH}/libRobotecGPULidar.so")

add_library(RGLServerPluginManager SHARED src/RGLServerPluginManager.cc src/Mesh.cc src/Utils.cc src/Scene.cc)
target_link_libraries(RGLServerPluginManager
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::ignition-plugin1
    ${RobotecGPULidar}
)

add_library(RGLServerPluginInstance SHARED src/RGLServerPluginInstance.cc src/Lidar.cc src/Utils.cc)
target_link_libraries(RGLServerPluginInstance
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::ignition-plugin1
    ${RobotecGPULidar}
)
