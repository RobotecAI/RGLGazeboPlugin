// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ignition/gazebo/components/SystemPluginInfo.hh>
#include <ignition/plugin/Register.hh>

#include "RGLServerPluginManager.hh"

#define PARAM_DO_IGNORE_ENTITIES_IN_LIDAR_LINK_ID "do_ignore_entities_in_lidar_link"

IGNITION_ADD_PLUGIN(
    rgl::RGLServerPluginManager,
    ignition::gazebo::System,
    rgl::RGLServerPluginManager::ISystemConfigure,
    rgl::RGLServerPluginManager::ISystemPostUpdate
)

using namespace std::placeholders;

namespace rgl
{

void RGLServerPluginManager::Configure(
        const ignition::gazebo::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        ignition::gazebo::EntityComponentManager& ecm,
        ignition::gazebo::EventManager& evm)
{
    ValidateRGLVersion();
    if (!CheckRGL(rgl_configure_logging(RGL_LOG_LEVEL_ERROR, nullptr, true))) {
        ignerr << "Failed to configure RGL logging.\n";
    }

    if (sdf->HasElement(PARAM_DO_IGNORE_ENTITIES_IN_LIDAR_LINK_ID)) {
        doIgnoreEntitiesInLidarLink = sdf->Get<bool>(PARAM_DO_IGNORE_ENTITIES_IN_LIDAR_LINK_ID);
    }
}

void RGLServerPluginManager::PostUpdate(
        const ignition::gazebo::UpdateInfo& info,
        const ignition::gazebo::EntityComponentManager& ecm)
{
    ecm.EachNew<>
            ([this, &ecm](auto&& entity) {
                return RegisterNewLidarCb(entity, ecm);
            });

    ecm.EachNew<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            ([this](auto&& entity, auto&& visual, auto&& geometry) {
                return LoadEntityToRGLCb(entity, visual, geometry);
            });

    ecm.EachNew<ignition::gazebo::components::LaserRetro>
            ([this](auto&& entity, auto&& laserRetro) {
                return SetLaserRetroCb(entity, laserRetro);
            });

    ecm.EachRemoved<>
            ([this, &ecm](auto&& entity) {
                return UnregisterLidarCb(entity, ecm);
            });

    ecm.EachRemoved<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            ([this](auto&& entity, auto&& visual, auto&& geometry) {
                return RemoveEntityFromRGLCb(entity, visual, geometry);
            });

    UpdateRGLEntityTransforms(ecm);
}

}  // namespace rgl
