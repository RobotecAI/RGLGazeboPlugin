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

#pragma once

#include "rgl/api/core.h"

#include <ignition/common/MeshManager.hh>

#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/transport/Node.hh>

#define WORLD_ENTITY_ID 1

#define RGL_CHECK(call)                  \
do {                                     \
    rgl_status_t status = call;          \
    if (status != RGL_SUCCESS) {         \
        const char* msg;                 \
        rgl_get_last_error_string(&msg); \
        ignerr << msg;                   \
        exit(1);                         \
    }                                    \
} while(0)

using namespace std::literals::chrono_literals;

namespace rgl {
    class RGLServerPluginInstance :
            public ignition::gazebo::System,
            public ignition::gazebo::ISystemConfigure,
            public ignition::gazebo::ISystemPreUpdate,
            public ignition::gazebo::ISystemPostUpdate {

    public:
        RGLServerPluginInstance();

        ~RGLServerPluginInstance() override;

        // only called once, when plugin is being loaded
        void Configure(
                const ignition::gazebo::Entity& entity,
                const std::shared_ptr<const sdf::Element>& sdf,
                ignition::gazebo::EntityComponentManager& ecm,
                ignition::gazebo::EventManager& eventMgr) override;

        // called every time before physics update runs (can change entities)
        void PreUpdate(
                const ignition::gazebo::UpdateInfo& info,
                ignition::gazebo::EntityComponentManager& ecm) override;

        // called every time after physics runs (can't change entities)
        void PostUpdate(
                const ignition::gazebo::UpdateInfo& info,
                const ignition::gazebo::EntityComponentManager& ecm) override;

    private:
        ////////////////////////////////////////////// Variables /////////////////////////////////////////////

        int current_update = 0;

        int updates_between_raytraces = 0;

        int last_raytrace_update = 0;

        ignition::transport::Node::Publisher pointcloud_publisher;

        ignition::transport::Node node;

        ignition::gazebo::Entity gazebo_lidar = 0;

        int lidar_id = -1;

        bool lidar_exists = true;

        rgl_node_t node_use_rays = nullptr;
        rgl_node_t node_lidar_pose = nullptr;
        rgl_node_t node_raytrace = nullptr;
        rgl_node_t node_compact = nullptr;

        std::chrono::steady_clock::duration last_raytrace_time = 0ms;

        std::chrono::steady_clock::duration time_between_raytraces = 100ms;

        ////////////////////////////////////////////// Functions /////////////////////////////////////////////

        void CreateLidar(ignition::gazebo::Entity entity);

        void UpdateLidarPose(const ignition::gazebo::EntityComponentManager& ecm,
                             std::chrono::steady_clock::duration sim_time,
                             bool paused);

        void RayTrace(ignition::gazebo::EntityComponentManager& ecm,
                      std::chrono::steady_clock::duration sim_time,
                      bool paused);

        bool CheckLidarExists(ignition::gazebo::Entity entity);
    };
}
