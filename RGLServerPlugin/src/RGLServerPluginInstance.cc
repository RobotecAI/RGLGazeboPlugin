#include "RGLServerPluginInstance.hh"

#include <ignition/gazebo/components/SystemPluginInfo.hh>
#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
        rgl::RGLServerPluginInstance,
        ignition::gazebo::System,
        rgl::RGLServerPluginInstance::ISystemConfigure,
        rgl::RGLServerPluginInstance::ISystemPreUpdate,
        rgl::RGLServerPluginInstance::ISystemPostUpdate
)

using namespace rgl;
using namespace std::placeholders;

RGLServerPluginInstance::RGLServerPluginInstance() = default;

RGLServerPluginInstance::~RGLServerPluginInstance() = default;

void RGLServerPluginInstance::Configure(
        const ignition::gazebo::Entity& entity,
        const std::shared_ptr<const sdf::Element>&,
        ignition::gazebo::EntityComponentManager& ecm,
        ignition::gazebo::EventManager& evm) {

    CreateLidar(entity);
}

void RGLServerPluginInstance::PreUpdate(
        const ignition::gazebo::UpdateInfo& info,
        ignition::gazebo::EntityComponentManager& ecm) {

    ecm.EachRemoved<>(std::bind(&RGLServerPluginInstance::CheckLidarExists, this, _1));

    RayTrace(ecm, info.simTime, info.paused);
}

void RGLServerPluginInstance::PostUpdate(
        const ignition::gazebo::UpdateInfo& info,
        const ignition::gazebo::EntityComponentManager& ecm) {

    UpdateLidarPose(ecm);
}
