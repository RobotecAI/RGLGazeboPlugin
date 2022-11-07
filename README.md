# RGLGazeboPlugin
## Requirements:

OS: [Ubuntu v22.04 Focal Fossa](https://releases.ubuntu.com/20.04.5/?_ga=2.210010709.1162335333.1667845331-1529863968.1667845331)

Gazebo: [Fortress v6.12](https://gazebosim.org/docs/fortress/install)

RGL: [v0.11.0](https://github.com/RobotecAI/RobotecGPULidar/tree/v11)

### Installation:
From RGLGazeboPlugin directory:
```
mkdir build
cd build
cmake ..
make -j
cd ..
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build/RGLServerPlugin
export IGN_GUI_PLUGIN_PATH=`pwd`/build/RGLGuiPlugin
```
### demo:
```
cd test_world
ign gazebo -v 4 actor_world.sdf
```

1. load RGLGuiPlugin from the menu on the upper right
2. press the refresh button in the plugin
3. start the simulation by pressing play, the lidar hits should be visible in the GUI