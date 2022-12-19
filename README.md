![](docs/images/RGLGazeboPlugin_logo.png)

![](docs/images/RGL_vs_built_in_lidar.png)

# RGL Gazebo Plugin:

Created By [ROBOTEC.AI](https://robotec.ai/) brings [Robotec GPU Lidar](https://github.com/RobotecAI/RobotecGPULidar) to [Gazebo](https://gazebosim.org/home). Get ready for great performance (faster that the gpu_lidar sensor from Gazebo),
smooth pointcloud result (left image above presents RGL, right shows gpu_lidar), because each ray is simulated separately and don't forget about the option to simulate non-standard lidar patterns 
(the pattern can be whatever you want provided you are able to create transforms for each of the rays).

## Requirements:

OS: [Ubuntu 22.04 Focal Fossa](https://releases.ubuntu.com/20.04.5/?_ga=2.210010709.1162335333.1667845331-1529863968.1667845331)

Gazebo: [Fortress 6.12](https://gazebosim.org/docs/fortress/install)

GPU: [Nvidia Pascal](https://en.wikipedia.org/wiki/Pascal_(microarchitecture)) architecture or newer (preferably with RT cores)

### Installation:
From RGLGazeboPlugin directory:

```shell
mkdir build
cd build
cmake ..
make -j
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/RGLServerPlugin
export IGN_GUI_PLUGIN_PATH=`pwd`/RGLVisualize
```
### Demo:

![](docs/videos/prius.gif)

From RGLGazeboPlugin directory:
```shell
cd test_world
ign gazebo -v 4 prius_world_RGL.sdf
```

1. load RGL Visualize plugin from the menu on the upper right (3 vertical dots)
2. start the simulation by pressing play
3. the lidar hits should be visible in the GUI
4. You can control the car using the Teleop plugin (preferably changing the steering to keyboard and upping the speed to 15)

## Using the plugin:

For the plugin to work properly, we need to include both RGLServerPluginManager and RGLServerPluginInstance(s):

### How to include RGLServerPluginManager to your sdf:
```xml
<plugin filename="RGLServerPluginManager" name="rgl::RGLServerPluginManager"></plugin>
```
This is a global plugin and should be included only once per sdf, preferably inside the world entity. 
### How to include RGLServerPluginInstance to your sdf:
```xml
<plugin filename="RGLServerPluginInstance" name="rgl::RGLServerPluginInstance"></plugin>
```
Note that the lidar will be attached to the entity that the instance inclusion is inside and will ignore all children (recursively) of the entity as well as the entity that it is attached to.
### RGLServerPluginInstance settings:
```xml
<plugin filename="RGLServerPluginInstance" name="rgl::RGLServerPluginInstance">
    <range>245</range>
    <update_rate>10</update_rate>
    <always_on>false</always_on>
    <custom_pattern>
        <horizontal>
            <samples>3600</samples>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
            <samples>128</samples>
            <min_angle>-0.436332</min_angle>
            <max_angle>0.261799</max_angle>
        </vertical>
    </custom_pattern>
    <pattern>Alpha Prime</pattern>>
</plugin>
```
range - the maximum range that the hits will be registered

update_rate - the frequency at which the lidar will perform raycasting (in Hz)

always_on - determines if the lidar should be active when the simulation is paused

custom pattern - you can generate your own pattern by tweaking the angle and sample count (the samples are distributed evenly)

pattern - you can type in the name of a Lidar to use its firing pattern

## Adding your own pattern:
```c++
int rays = samples_vertical * samples_horizontal;
    std::vector<rgl_mat3x4f> ray_tf;

    ignition::math::Angle vertical_step((vertical_max - vertical_min) / static_cast<double>(samples_vertical));
    ignition::math::Angle horizontal_step((horizontal_max - horizontal_min) / static_cast<double>(samples_horizontal));

    ignition::math::Angle pitch(ignition::math::Angle::HalfPi - vertical_max);
    ignition::math::Angle jaw(horizontal_min);

    for (int i = 0; i < samples_vertical; ++i, pitch += vertical_step) {
        for (int j = 0; j < samples_horizontal; ++j, jaw += horizontal_step) {
            AddToRayTf(ray_tf, ignition::math::Angle::Zero, pitch, jaw);
        }
    }

    auto rgl_pose_matrix = RGLServerPluginManager::GetRglMatrix(gazebo_lidar, ecm);

    RGL_CHECK(rgl_node_rays_from_mat3x4f(&node_use_rays, ray_tf.data(), rays));
```
You have to edit the Lidar.cc file and in the function CreateLidar, in the place of ray_tf provide your own array of ray transforms and also the rays variable has to be updated to store your array's length.