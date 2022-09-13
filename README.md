# RGLGazeboPlugin
### Installation:
From RGLGazeboPlugin directory:
```
mkdir build
cd build
cmake ..
make
cd ..
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build
```
### test world:
```
cd test_world
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ign gazebo -v 4 actor_world.sdf
```

1. load RGLVisualize plugin from the menu on the upper right
2. press the refresh button in the plugin (the selected topic should now be "/point_cloud")
3. start the simulation by pressing play, the lidar hits should be visible in the GUI