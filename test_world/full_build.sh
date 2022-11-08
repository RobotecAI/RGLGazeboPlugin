cd ..
rm -rf build
mkdir build
cd build
cmake ..
make -j
cd ..
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build/RGLServerPlugin
export IGN_GUI_PLUGIN_PATH=`pwd`/build/RGLGuiPlugin
cd test_world