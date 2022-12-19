![](docs/images/RGLGazeboPlugin_logo.png)

## Requirements:

OS: [Ubuntu 22.04 Focal Fossa](https://releases.ubuntu.com/20.04.5/?_ga=2.210010709.1162335333.1667845331-1529863968.1667845331)

Gazebo: [Fortress 6.12](https://gazebosim.org/docs/fortress/install)

GPU: [Nvidia Pascal](https://en.wikipedia.org/wiki/Pascal_(microarchitecture)) architecture or newer (preferably with RT cores)

### Installation:
From RGLGazeboPlugin directory:

```
mkdir build
cd build
cmake ..
make -j
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/RGLServerPlugin
export IGN_GUI_PLUGIN_PATH=`pwd`/RGLVisualize
```
### Demo:

![](docs/videos/prius.gif)

From RGLGazeboPlugin/test_world directory:
```
ign gazebo -v 4 prius_world_RGL.sdf
```

1. load RGLVisualize plugin from the menu on the upper right (3 vertical dots)
2. start the simulation by pressing play
3. the lidar hits should be visible in the GUI
4. You can control the car using the Teleop plugin (preferably changing the steering to keyboard and upping the speed to 15)