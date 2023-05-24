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

#include <cstdio>

#include "RGLServerPluginInstance.hh"
#include "Utils.hh"

#define PARAM_UPDATE_RATE_ID "update_rate"
#define PARAM_RANGE_ID "range"
#define PARAM_TOPIC_ID "topic"
#define PARAM_FRAME_ID "frame"
#define PARAM_UPDATE_ON_PAUSED_SIM_ID "update_on_paused_sim"

namespace rgl
{

bool RGLServerPluginInstance::LoadConfiguration(const std::shared_ptr<const sdf::Element>& sdf)
{
    // Required parameters
    if (!sdf->HasElement(PARAM_UPDATE_RATE_ID)) {
        ignerr << "No '" << PARAM_UPDATE_RATE_ID << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    if (!sdf->HasElement(PARAM_RANGE_ID)) {
        ignerr << "No '" << PARAM_RANGE_ID << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    if (!sdf->HasElement(PARAM_TOPIC_ID)) {
        ignerr << "No '" << PARAM_TOPIC_ID << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    if (!sdf->HasElement(PARAM_FRAME_ID)) {
        ignerr << "No '" << PARAM_FRAME_ID << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    // Optional parameters
    if (!sdf->HasElement(PARAM_UPDATE_ON_PAUSED_SIM_ID)) {
        ignwarn << "No '" << PARAM_UPDATE_ON_PAUSED_SIM_ID << "' parameter specified for the RGL lidar. "
                << "Using default value: " << updateOnPausedSim << "\n";
    } else {
        updateOnPausedSim = sdf->Get<bool>(PARAM_UPDATE_ON_PAUSED_SIM_ID);
    }

    // Load configuration
    float updateRateHz = sdf->Get<float>(PARAM_UPDATE_RATE_ID);
    raytraceIntervalTime = std::chrono::microseconds(static_cast<int64_t>(1e6 / updateRateHz));
    lidarRange = sdf->Get<float>(PARAM_RANGE_ID);
    topicName = sdf->Get<std::string>(PARAM_TOPIC_ID);
    frameId = sdf->Get<std::string>(PARAM_FRAME_ID);

    if (!LidarPatternLoader::Load(sdf, lidarPattern)) {
        return false;
    }
    // Resize container for result point cloud
    resultPointCloud.resize(lidarPattern.size());

    return true;
}

void RGLServerPluginInstance::CreateLidar(ignition::gazebo::Entity entity,
                                          ignition::gazebo::EntityComponentManager& ecm)
{
    thisLidarEntity = entity;

    rgl_mat3x4f identity = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0
    };

    if (!CheckRGL(rgl_node_rays_from_mat3x4f(&rglNodeUseRays, lidarPattern.data(), lidarPattern.size())) ||
        !CheckRGL(rgl_node_rays_transform(&rglNodeLidarPose, &identity)) ||
        !CheckRGL(rgl_node_raytrace(&rglNodeRaytrace, nullptr, lidarRange)) ||
        !CheckRGL(rgl_node_points_compact(&rglNodeCompact)) ||
        !CheckRGL(rgl_node_points_transform(&rglNodeToLidarFrame, &identity))) {

        ignerr << "Failed to create RGL nodes when initializing lidar. Disabling plugin.\n";
        return;
    }

    if (!CheckRGL(rgl_graph_node_add_child(rglNodeUseRays, rglNodeLidarPose)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeLidarPose, rglNodeRaytrace)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeRaytrace, rglNodeCompact)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeCompact, rglNodeToLidarFrame))) {
        
        ignerr << "Failed to connect RGL nodes when initializing lidar. Disabling plugin.\n";
        return;
    }

    pointCloudPublisher = gazeboNode.Advertise<ignition::msgs::PointCloudPacked>(topicName);
    pointCloudWorldPublisher = gazeboNode.Advertise<ignition::msgs::PointCloudPacked>(topicName + worldTopicPostfix);

    isLidarInitialized = true;
}

void RGLServerPluginInstance::UpdateLidarPose(const ignition::gazebo::EntityComponentManager& ecm)
{
    ignition::math::Pose3<double> ignLidarToWorld = FindWorldPose(thisLidarEntity, ecm);
    ignition::math::Pose3<double> ignWorldToLidar = ignLidarToWorld.Inverse();
    rgl_mat3x4f rglLidarToWorld = IgnPose3dToRglMatrix(ignLidarToWorld);
    rgl_mat3x4f rglWorldToLidar = IgnPose3dToRglMatrix(ignWorldToLidar);
    CheckRGL(rgl_node_rays_transform(&rglNodeLidarPose, &rglLidarToWorld));
    CheckRGL(rgl_node_points_transform(&rglNodeToLidarFrame, &rglWorldToLidar));
}

bool RGLServerPluginInstance::ShouldRayTrace(std::chrono::steady_clock::duration simTime,
                                             bool paused)
{
    if (!isLidarInitialized) {
        return false;
    }

    if (paused) {
        ++onPausedSimUpdateCounter;
        if (!updateOnPausedSim) {
            return false;
        }
        if (onPausedSimUpdateCounter < onPausedSimRaytraceInterval) {
            return false;
        }
        onPausedSimUpdateCounter = 0;
        return true;
    }

    // Simulation running
    if (simTime < lastRaytraceTime + raytraceIntervalTime) {
        return false;
    }
    return true;
}

void RGLServerPluginInstance::RayTrace(std::chrono::steady_clock::duration simTime)
{
    lastRaytraceTime = simTime;

    if (!CheckRGL(rgl_graph_run(rglNodeRaytrace))) {
        ignerr << "Failed to perform raytrace.\n";
        return;
    }

    int32_t hitpointCount = 0;
    if (!CheckRGL(rgl_graph_get_result_size(rglNodeToLidarFrame, RGL_FIELD_XYZ_F32, &hitpointCount, nullptr)) ||
        !CheckRGL(rgl_graph_get_result_data(rglNodeToLidarFrame, RGL_FIELD_XYZ_F32, resultPointCloud.data()))) {

        ignerr << "Failed to get result data from RGL lidar.\n";
        return;
    }

    auto msg = CreatePointCloudMsg(simTime, frameId, hitpointCount);
    pointCloudPublisher.Publish(msg);

    if (pointCloudWorldPublisher.HasConnections()) {
        if (!CheckRGL(rgl_graph_get_result_size(rglNodeToLidarFrame, RGL_FIELD_XYZ_F32, &hitpointCount, nullptr)) ||
            !CheckRGL(rgl_graph_get_result_data(rglNodeCompact, RGL_FIELD_XYZ_F32, resultPointCloud.data()))) {

            ignerr << "Failed to get visualization data from RGL lidar.\n";
            return;
        }
        auto worldMsg = CreatePointCloudMsg(simTime, worldFrameId, hitpointCount);
        pointCloudWorldPublisher.Publish(worldMsg);
    }
}

ignition::msgs::PointCloudPacked RGLServerPluginInstance::CreatePointCloudMsg(std::chrono::steady_clock::duration simTime, std::string frame, int hitpointCount)
{
    ignition::msgs::PointCloudPacked outMsg;
    ignition::msgs::InitPointCloudPacked(outMsg, frame, false,
                                         {{"xyz", ignition::msgs::PointCloudPacked::Field::FLOAT32}});
    outMsg.mutable_data()->resize(hitpointCount * outMsg.point_step());
    *outMsg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(simTime);
    outMsg.set_height(1);
    outMsg.set_width(hitpointCount);

    ignition::msgs::PointCloudPackedIterator<float> xIterWorld(outMsg, "x");
    memcpy(&(*xIterWorld), resultPointCloud.data(), hitpointCount * sizeof(rgl_vec3f));
    return outMsg;
}

void RGLServerPluginInstance::DestroyLidar()
{
    if (!isLidarInitialized) {
        return;
    }

    if (!CheckRGL(rgl_graph_destroy(rglNodeRaytrace))) {
        ignerr << "Failed to destroy RGL lidar.\n";
    }
    // Reset publishers
    pointCloudPublisher = ignition::transport::Node::Publisher();
    pointCloudWorldPublisher = ignition::transport::Node::Publisher();
    isLidarInitialized = false;
}

}  // namespace rgl
