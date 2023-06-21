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

#include <gz/sim/components/SystemPluginInfo.hh>
#include <gz/plugin/Register.hh>

#include "RGLServerPluginManager.hh"

GZ_ADD_PLUGIN(
    rgl::RGLServerPluginManager,
    gz::sim::System,
    rgl::RGLServerPluginManager::ISystemConfigure,
    rgl::RGLServerPluginManager::ISystemPostUpdate
)

using namespace std::placeholders;

namespace rgl
{

void RGLServerPluginManager::Configure(
        const gz::sim::Entity& entity,
        const std::shared_ptr<const sdf::Element>&,
        gz::sim::EntityComponentManager& ecm,
        gz::sim::EventManager& evm)
{
    ValidateRGLVersion();
    if (!CheckRGL(rgl_configure_logging(RGL_LOG_LEVEL_ERROR, nullptr, true))) {
        ignerr << "Failed to configure RGL logging.\n";
    }
}

void RGLServerPluginManager::PostUpdate(
        const gz::sim::UpdateInfo& info,
        const gz::sim::EntityComponentManager& ecm)
{
    ecm.EachNew<>([&](const gz::sim::Entity& entity)-> bool {
        return RegisterNewLidarCb(entity, ecm);});

    ecm.EachNew<gz::sim::components::Visual, gz::sim::components::Geometry>
            (std::bind(&RGLServerPluginManager::LoadEntityToRGLCb, this, _1, _2, _3));

    ecm.EachRemoved<>([&](const gz::sim::Entity& entity)-> bool {
        return UnregisterLidarCb(entity, ecm);});

    ecm.EachRemoved<gz::sim::components::Visual, gz::sim::components::Geometry>
            (std::bind(&RGLServerPluginManager::RemoveEntityFromRGLCb, this, _1, _2, _3));

    UpdateRGLEntityPoses(ecm);
}

}  // namespace rgl
