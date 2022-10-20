#include "server/ServerPlugin.hh"

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/VisualCmd.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
        rgl::ServerPlugin,
        ignition::gazebo::System,
        rgl::ServerPlugin::ISystemConfigure,
        rgl::ServerPlugin::ISystemPreUpdate,
        rgl::ServerPlugin::ISystemPostUpdate
)

using namespace rgl;
using namespace std::literals::chrono_literals;
using namespace std::placeholders;

ServerPlugin::ServerPlugin() = default;

ServerPlugin::~ServerPlugin() = default;

void ServerPlugin::Configure(
        const ignition::gazebo::Entity& entity,
        const std::shared_ptr<const sdf::Element>&,
        ignition::gazebo::EntityComponentManager& ecm,
        ignition::gazebo::EventManager& evm) {

    CreateLidar();

    ignmsg << "attached to: " << entity << std::endl;
    gazebo_lidar = entity;
    static int next_free_id = 0;
    lidar_id = next_free_id;
    next_free_id++;

    for (auto descendant: ecm.Descendants(gazebo_lidar)) {
        lidar_ignore.insert(descendant);
    };

    ecm.Each<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            (std::bind(&ServerPlugin::LoadEntityToRGL, this, _1, _2, _3));
}

void ServerPlugin::PreUpdate(
        const ignition::gazebo::UpdateInfo& info,
        ignition::gazebo::EntityComponentManager& ecm) {

    if (!gazebo_lidar_exists) return;

    if (!ray_trace) return;
    ray_trace = false;

    RayTrace(ecm);
}

void ServerPlugin::PostUpdate(
        const ignition::gazebo::UpdateInfo& info,
        const ignition::gazebo::EntityComponentManager& ecm) {

    if (!gazebo_lidar_exists) return;

    ecm.EachNew<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            (std::bind(&ServerPlugin::LoadEntityToRGL, this, _1, _2, _3));

    ecm.EachRemoved<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            (std::bind(&ServerPlugin::RemoveEntityFromRGL, this, _1, _2, _3));

    if (info.simTime < last_update + 100ms) return;
    last_update = info.simTime;
    ray_trace = true;

    UpdateRGLEntityPose(ecm);

    UpdateLidarPose(ecm);
}