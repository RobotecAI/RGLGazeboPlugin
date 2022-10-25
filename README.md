# RGLGazeboPlugin
## Warning:

Currently tested only on ubuntu 20.04 (ubuntu 22 has faulty gazebo fortress atm)

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