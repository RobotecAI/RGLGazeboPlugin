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
#include "LidarPatternLoader.hh"

#include <ignition/common/MeshManager.hh>

#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/transport/Node.hh>

namespace rgl
{

class RGLServerPluginInstance :
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate
{
public:
    RGLServerPluginInstance() = default;

    ~RGLServerPluginInstance() override = default;

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
    bool LoadConfiguration(const std::shared_ptr<const sdf::Element>& sdf);
    void CreateLidar(ignition::gazebo::Entity entity,
                     ignition::gazebo::EntityComponentManager& ecm);

    void UpdateLidarPose(const ignition::gazebo::EntityComponentManager& ecm);

    bool ShouldRayTrace(std::chrono::steady_clock::duration sim_time,
                        bool paused);
    void RayTrace(std::chrono::steady_clock::duration sim_time);

    bool FetchLaserScanResult();
    bool FetchPointCloudResult(rgl_node_t formatNode);

    ignition::msgs::PointCloudPacked CreatePointCloudMsg(std::chrono::steady_clock::duration sim_time, const std::string& frame);
    ignition::msgs::LaserScan CreateLaserScanMsg(std::chrono::steady_clock::duration sim_time, const std::string& frame);

    void DestroyLidar();

    std::string topicName;
    std::string frameId;
    rgl_vec2f lidarMinMaxRange;
    ignition::math::Angle scanHMin;
    ignition::math::Angle scanHMax;
    int scanHSamples;
    std::vector<rgl_mat3x4f> lidarPattern;

    struct ResultPointCloud
    {
        std::vector<char> data{};
        int32_t hitPointCount{};
        const std::size_t pointSize{sizeof(rgl_vec3f) + sizeof(float)};  // Based on rglFields
        const std::vector<rgl_field_t> rglFields = {
                RGL_FIELD_XYZ_VEC3_F32,
                RGL_FIELD_LASER_RETRO_F32
        };
    } resultPointCloud{};

    struct ResultLaserScan
    {
        std::vector<float> distances{};
        std::vector<float> intensities{};
        // No hitPointCount needed because LaserScan message contains non-hits also
        const std::vector<rgl_field_t> rglFields = {
            RGL_FIELD_DISTANCE_F32,
            RGL_FIELD_LASER_RETRO_F32
        };
    } resultLaserScan{};

    bool updateOnPausedSim = false;
    bool publishLaserScan = false;

    ignition::gazebo::Entity thisLidarEntity;
    ignition::transport::Node::Publisher pointCloudPublisher;
    ignition::transport::Node::Publisher laserScanPublisher;
    ignition::transport::Node::Publisher pointCloudWorldPublisher;
    ignition::transport::Node gazeboNode;

    rgl_node_t rglNodeUseRays = nullptr;
    rgl_node_t rglNodeLidarPose = nullptr;
    rgl_node_t rglNodeSetRange = nullptr;
    rgl_node_t rglNodeRaytrace = nullptr;
    rgl_node_t rglNodeCompact = nullptr;
    rgl_node_t rglNodeYieldLaserScan = nullptr;
    rgl_node_t rglNodeFormatPointCloudSensor = nullptr;
    rgl_node_t rglNodeFormatPointCloudWorld = nullptr;
    rgl_node_t rglNodeToLidarFrame = nullptr;

    std::chrono::steady_clock::duration raytraceIntervalTime;
    std::chrono::steady_clock::duration lastRaytraceTime{0};

    bool isLidarInitialized = false;

    int onPausedSimUpdateCounter = 0;
    const int onPausedSimRaytraceInterval = 100;

    const std::string worldFrameId = "world";
    const std::string worldTopicPostfix = "/world";
};

}  // namespace rgl
