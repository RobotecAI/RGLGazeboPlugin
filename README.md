# RGLGazeboPlugin
### Installation:

mkdir build

cd build

cmake ..

make

cd ..

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build

### test world:

cd test_world

ign gazebo -v 4 actor_world.sdf

After pressing play button the parameters of the minecart mesh should be visible in the terminal as [Msg]