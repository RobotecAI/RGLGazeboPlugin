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
        const std::shared_ptr<const sdf::Element>& sdf,
        ignition::gazebo::EntityComponentManager& ecm,
        ignition::gazebo::EventManager&) {

    if (LoadConfiguration(sdf)) {
        CreateLidar(entity, ecm);
    };
}

void RGLServerPluginInstance::PreUpdate(
        const ignition::gazebo::UpdateInfo& info,
        ignition::gazebo::EntityComponentManager& ecm) {

    if (ShouldRayTrace(info.simTime, info.paused)) {
        UpdateLidarPose(ecm);
        RayTrace(info.simTime);
    }
}

void RGLServerPluginInstance::PostUpdate(
        const ignition::gazebo::UpdateInfo& info,
        const ignition::gazebo::EntityComponentManager& ecm) {

    ecm.EachRemoved<>([&](const ignition::gazebo::Entity& entity)-> bool {
        if (entity == thisLidarEntity) {
            DestroyLidar();
        }
        return true;
    });
}
