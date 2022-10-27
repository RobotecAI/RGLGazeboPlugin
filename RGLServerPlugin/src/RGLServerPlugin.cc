#include "RGLServerPlugin.hh"

#include <ignition/gazebo/components/SystemPluginInfo.hh>
#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
        rgl::RGLServerPlugin,
        ignition::gazebo::System,
        rgl::RGLServerPlugin::ISystemConfigure,
        rgl::RGLServerPlugin::ISystemPreUpdate,
        rgl::RGLServerPlugin::ISystemPostUpdate
)

using namespace rgl;
using namespace std::literals::chrono_literals;
using namespace std::placeholders;

RGLServerPlugin::RGLServerPlugin() = default;

RGLServerPlugin::~RGLServerPlugin() = default;

void RGLServerPlugin::Configure(
        const ignition::gazebo::Entity& entity,
        const std::shared_ptr<const sdf::Element>&,
        ignition::gazebo::EntityComponentManager& ecm,
        ignition::gazebo::EventManager& evm) {

    ignmsg << "attached to: " << entity << std::endl;
    gazebo_lidar = entity;
    static int next_free_id = 0;
    lidar_id = next_free_id;
    next_free_id++;

    CreateLidar();

    for (auto descendant: ecm.Descendants(gazebo_lidar)) {
        lidar_ignore.insert(descendant);
    }

    ecm.Each<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            (std::bind(&RGLServerPlugin::LoadEntityToRGL, this, _1, _2, _3));

    for (auto component_type : ecm.ComponentTypes(gazebo_lidar)) {
        std::cout << ignition::gazebo::components::Factory::Instance()->Name(component_type) << " -> " << component_type << std::endl;
        if (component_type == 3246420869490501367) {
            auto data = ecm.ComponentData<ignition::gazebo::components::SystemPluginInfo>(gazebo_lidar);
            auto plugins = data->plugins();
            for (const auto& plugin : plugins) {
                std::cout << plugin.name() << std::endl;
            }
        }
    }
}

void RGLServerPlugin::PreUpdate(
        const ignition::gazebo::UpdateInfo& info,
        ignition::gazebo::EntityComponentManager& ecm) {

    if (!gazebo_lidar_exists) return;

    if (!ray_trace) return;
    ray_trace = false;

    RayTrace(ecm);
}

void RGLServerPlugin::PostUpdate(
        const ignition::gazebo::UpdateInfo& info,
        const ignition::gazebo::EntityComponentManager& ecm) {

    if (!gazebo_lidar_exists) return;

    ecm.EachNew<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            (std::bind(&RGLServerPlugin::LoadEntityToRGL, this, _1, _2, _3));

    ecm.EachRemoved<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            (std::bind(&RGLServerPlugin::RemoveEntityFromRGL, this, _1, _2, _3));

    if (info.simTime < last_update + 100ms) return;
    last_update = info.simTime;
    ray_trace = true;

    UpdateRGLEntityPose(ecm);

    UpdateLidarPose(ecm);
}
