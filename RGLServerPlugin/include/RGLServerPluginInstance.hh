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

#include <filesystem>

#include "rgl/api/core.h"

#include <ignition/common/MeshManager.hh>

#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/transport/Node.hh>

#define WORLD_ENTITY_ID 1

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

        std::string ray_tf_file_path;
        bool always_on = false;
        std::vector<float> layer_angles;
        float range = 1000.0f;
        ignition::math::Angle horizontal_max = ignition::math::Angle::Pi;
        ignition::math::Angle horizontal_min = ignition::math::Angle::Pi * -1;
        ignition::math::Angle vertical_max = ignition::math::Angle::HalfPi;
        ignition::math::Angle vertical_min = ignition::math::Angle::HalfPi * -1;
        int samples_vertical = 100;
        int samples_horizontal = 2500;

        int64_t current_update = 0;
        int updates_between_raytraces = 100;
        bool before_first_raytrace = true;
        int64_t last_raytrace_update = 0;

        ignition::transport::Node::Publisher pointcloud_publisher;
        ignition::transport::Node node;

        ignition::gazebo::Entity gazebo_lidar;
        int lidar_id;
        bool lidar_exists = false;

        rgl_node_t node_use_rays = nullptr;
        rgl_node_t node_lidar_pose = nullptr;
        rgl_node_t node_raytrace = nullptr;
        rgl_node_t node_compact = nullptr;

        std::chrono::steady_clock::duration last_raytrace_time = 0ms;
        std::chrono::steady_clock::duration time_between_raytraces = 100ms;

        std::vector<rgl_vec3f> results{rgl_vec3f{}};

        std::unordered_map<std::string, std::string> pattern_names = {
                {"Alpha Prime", "VelodyneVLS128"},
                {"Puck", "VelodyneVLP16"},
                {"Ultra Puck", "VelodyneVLP32C"},
                {"OS1 64", "OusterOS1_64"},
                {"Pandar64", "HesaiPandarQT64"},
                {"Pandar40P", "HesaiPandar40P"}
        };

        ////////////////////////////////////////////// Functions /////////////////////////////////////////////

        template<typename T>
        static std::vector<T> loadVector(const std::filesystem::path& path);

        void SetPatternFromName(const std::string& name);

        static void AddToRayTf(std::vector<rgl_mat3x4f>& ray_tf,
                        const ignition::math::Angle& roll,
                        const ignition::math::Angle& pitch,
                        const ignition::math::Angle& jaw);

        void LoadConfiguration(const std::shared_ptr<const sdf::Element>& sdf);

        void CreateLidar(ignition::gazebo::Entity entity,
                         ignition::gazebo::EntityComponentManager& ecm);

        void UpdateLidarPose(const ignition::gazebo::EntityComponentManager& ecm);

        bool ShouldRayTrace(std::chrono::steady_clock::duration sim_time,
                                                     bool paused);

        void RayTrace(std::chrono::steady_clock::duration sim_time);

        bool CheckLidarExists(ignition::gazebo::Entity entity);
    };
}
