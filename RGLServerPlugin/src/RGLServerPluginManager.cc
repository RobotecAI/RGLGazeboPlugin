#include "RGLServerPluginManager.hh"

#include <ignition/gazebo/components/SystemPluginInfo.hh>
#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
        rgl::RGLServerPluginManager,
        ignition::gazebo::System,
        rgl::RGLServerPluginManager::ISystemConfigure,
        rgl::RGLServerPluginManager::ISystemPreUpdate,
        rgl::RGLServerPluginManager::ISystemPostUpdate
)

using namespace rgl;
using namespace std::placeholders;

RGLServerPluginManager::RGLServerPluginManager() = default;

RGLServerPluginManager::~RGLServerPluginManager() = default;

void RGLServerPluginManager::Configure(
        const ignition::gazebo::Entity& entity,
        const std::shared_ptr<const sdf::Element>&,
        ignition::gazebo::EntityComponentManager& ecm,
        ignition::gazebo::EventManager& evm) {

    ecm.Each<>([&](const ignition::gazebo::Entity& entity)-> bool {
                return CheckNewLidarsCb(entity, ecm);});

    ecm.Each<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            (std::bind(&RGLServerPluginManager::LoadEntityToRGLCb, this, _1, _2, _3));
}

void RGLServerPluginManager::PreUpdate(
        const ignition::gazebo::UpdateInfo& info,
        ignition::gazebo::EntityComponentManager& ecm) {
}

void RGLServerPluginManager::PostUpdate(
        const ignition::gazebo::UpdateInfo& info,
        const ignition::gazebo::EntityComponentManager& ecm) {

    ecm.EachNew<>([&](const ignition::gazebo::Entity& entity)-> bool {
        return CheckNewLidarsCb(entity, ecm);});

    ecm.EachNew<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            (std::bind(&RGLServerPluginManager::LoadEntityToRGLCb, this, _1, _2, _3));

    ecm.EachRemoved<>([&](const ignition::gazebo::Entity& entity)-> bool {
        return CheckRemovedLidarsCb(entity, ecm);});

    ecm.EachRemoved<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            (std::bind(&RGLServerPluginManager::RemoveEntityFromRGLCb, this, _1, _2, _3));

    UpdateRGLEntityPose(ecm);
}
