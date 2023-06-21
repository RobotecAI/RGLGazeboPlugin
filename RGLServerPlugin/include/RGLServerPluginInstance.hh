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

#include <gz/common/MeshManager.hh>

#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/System.hh>

#include <gz/transport/Node.hh>

namespace rgl
{

class RGLServerPluginInstance :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
    RGLServerPluginInstance() = default;

    ~RGLServerPluginInstance() override = default;

    // only called once, when plugin is being loaded
    void Configure(
        const gz::sim::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        gz::sim::EntityComponentManager& ecm,
        gz::sim::EventManager& eventMgr) override;

    // called every time before physics update runs (can change entities)
    void PreUpdate(
            const gz::sim::UpdateInfo& info,
            gz::sim::EntityComponentManager& ecm) override;

    // called every time after physics runs (can't change entities)
    void PostUpdate(
            const gz::sim::UpdateInfo& info,
            const gz::sim::EntityComponentManager& ecm) override;

private:
    bool LoadConfiguration(const std::shared_ptr<const sdf::Element>& sdf);
    void CreateLidar(gz::sim::Entity entity,
                     gz::sim::EntityComponentManager& ecm);

    void UpdateLidarPose(const gz::sim::EntityComponentManager& ecm);

    bool ShouldRayTrace(std::chrono::steady_clock::duration sim_time,
                        bool paused);
    void RayTrace(std::chrono::steady_clock::duration sim_time);

    gz::msgs::PointCloudPacked CreatePointCloudMsg(std::string frame, int hitpointCount);

    void DestroyLidar();

    std::string topicName;
    std::string frameId;
    float lidarRange;
    std::vector<rgl_mat3x4f> lidarPattern;
    std::vector<rgl_vec3f> resultPointCloud;

    bool updateOnPausedSim = false;

    gz::sim::Entity thisLidarEntity;
    gz::transport::Node::Publisher pointCloudPublisher;
    gz::transport::Node::Publisher pointCloudWorldPublisher;
    gz::transport::Node gazeboNode;

    rgl_node_t rglNodeUseRays = nullptr;
    rgl_node_t rglNodeLidarPose = nullptr;
    rgl_node_t rglNodeRaytrace = nullptr;
    rgl_node_t rglNodeCompact = nullptr;
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
