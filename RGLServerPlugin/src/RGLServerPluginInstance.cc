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

#include "RGLServerPluginInstance.hh"

GZ_ADD_PLUGIN(
        rgl::RGLServerPluginInstance,
        gz::sim::System,
        rgl::RGLServerPluginInstance::ISystemConfigure,
        rgl::RGLServerPluginInstance::ISystemPreUpdate,
        rgl::RGLServerPluginInstance::ISystemPostUpdate
)

using namespace std::placeholders;

namespace rgl
{

void RGLServerPluginInstance::Configure(
        const gz::sim::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        gz::sim::EntityComponentManager& ecm,
        gz::sim::EventManager&)
{
    if (LoadConfiguration(sdf)) {
        CreateLidar(entity, ecm);
    };
}

void RGLServerPluginInstance::PreUpdate(
        const gz::sim::UpdateInfo& info,
        gz::sim::EntityComponentManager& ecm)
{
    if (ShouldRayTrace(info.simTime, info.paused)) {
        UpdateLidarPose(ecm);
        RayTrace(info.simTime);
    }
}

void RGLServerPluginInstance::PostUpdate(
        const gz::sim::UpdateInfo& info,
        const gz::sim::EntityComponentManager& ecm)
{
    ecm.EachRemoved<>([&](const gz::sim::Entity& entity)-> bool {
        if (entity == thisLidarEntity) {
            DestroyLidar();
        }
        return true;
    });
}

}  // namespace rgl
