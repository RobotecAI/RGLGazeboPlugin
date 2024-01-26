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

    // Check for 2d pattern and set LaserScan
    if (sdf->HasElement("pattern_lidar2d")) {
        ignmsg << "Lidar is 2D, switching to publish LaserScan messages";
        publishLaserScan = true;
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

    // Resize containers for result
    resultDistances.resize(lidarPattern.size());
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

    // set desired fields for API call
    std::vector<rgl_field_t> yieldFields;
    yieldFields.push_back(RGL_FIELD_XYZ_F32);

    if (publishLaserScan) {
        yieldFields.push_back(RGL_FIELD_DISTANCE_F32);
    }

    if (!CheckRGL(rgl_node_rays_from_mat3x4f(&rglNodeUseRays, lidarPattern.data(), lidarPattern.size())) ||
        !CheckRGL(rgl_node_rays_transform(&rglNodeLidarPose, &identity)) ||
        !CheckRGL(rgl_node_raytrace(&rglNodeRaytrace, nullptr, lidarRange)) ||
        !CheckRGL(rgl_node_points_yield(&rglNodeYield, yieldFields.data(), yieldFields.size())) ||
        !CheckRGL(rgl_node_points_transform(&rglNodeToLidarFrame, &identity))) {

        ignerr << "Failed to create RGL nodes when initializing lidar. Disabling plugin.\n";
        return;
    }

    if (!CheckRGL(rgl_graph_node_add_child(rglNodeUseRays, rglNodeLidarPose)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeLidarPose, rglNodeRaytrace)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeRaytrace, rglNodeYield)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeYield, rglNodeToLidarFrame))) {
        
        ignerr << "Failed to connect RGL nodes when initializing lidar. Disabling plugin.\n";
        return;
    }

    if (!publishLaserScan) {
        ignmsg << "Topic: " << topicName << " is PointCloudPacked.\n";
        pointCloudPublisher = gazeboNode.Advertise<ignition::msgs::PointCloudPacked>(topicName);
    } else {
        ignmsg << "Topic: " << topicName << " is LaserScan.\n";
        laserScanPublisher = gazeboNode.Advertise<ignition::msgs::LaserScan>(topicName);
    }
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

    if (!publishLaserScan) {
        if (!CheckRGL(rgl_graph_get_result_size(rglNodeToLidarFrame, RGL_FIELD_XYZ_F32, &hitpointCount, nullptr)) ||
            !CheckRGL(rgl_graph_get_result_data(rglNodeToLidarFrame, RGL_FIELD_XYZ_F32, resultPointCloud.data()))) {

            ignerr << "Failed to get result data from RGL lidar.\n";
            return;
            }
    } else {
        if (!CheckRGL(rgl_graph_get_result_size(rglNodeToLidarFrame, RGL_FIELD_DISTANCE_F32, &hitpointCount, nullptr)) ||
            !CheckRGL(rgl_graph_get_result_data(rglNodeToLidarFrame, RGL_FIELD_DISTANCE_F32, resultDistances.data()))) {

        ignerr << "Failed to get result distances from RGL lidar.\n";
        return;
        }
    }

    if (!publishLaserScan) {
        auto msg = CreatePointCloudMsg(simTime, frameId, hitpointCount);
        pointCloudPublisher.Publish(msg);
    } else {
        auto msg = CreateLaserScanMsg(simTime, frameId, hitpointCount);
        laserScanPublisher.Publish(msg);
    }

    if (pointCloudWorldPublisher.HasConnections()) {
        if (!CheckRGL(rgl_graph_get_result_size(rglNodeToLidarFrame, RGL_FIELD_XYZ_F32, &hitpointCount, nullptr)) ||
            !CheckRGL(rgl_graph_get_result_data(rglNodeYield, RGL_FIELD_XYZ_F32, resultPointCloud.data()))) {

            ignerr << "Failed to get visualization data from RGL lidar.\n";
            return;
        }
        auto worldMsg = CreatePointCloudMsg(simTime, worldFrameId, hitpointCount);
        pointCloudWorldPublisher.Publish(worldMsg);
    }
}

ignition::msgs::LaserScan RGLServerPluginInstance::CreateLaserScanMsg(std::chrono::steady_clock::duration simTime, std::string frame, int hitpointCount)
{
    ignition::msgs::LaserScan outMsg;
    ignition::math::Angle scanHMin;
    ignition::math::Angle scanHMax;
    int scanHSamples;

    *outMsg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(simTime);
    auto _frame = outMsg.mutable_header()->add_data();
    _frame->set_key("frame_id");
    _frame->add_value(frame);

    outMsg.set_frame(frame);
    outMsg.set_count(hitpointCount);

    outMsg.set_range_min(0.0);
    outMsg.set_range_max(lidarRange);

    scanHSamples = lidarPattern.size();

    ignition::math::Matrix3d matrix3DMin, matrix3DNext;
    ignition::math::Quaterniond quaternionMin, quaternionNext;
    ignition::math::Vector3d eulerMin, eulerNext;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            matrix3DMin(i, j) = lidarPattern[0].value[i][j];
            matrix3DNext(i, j) = lidarPattern[1].value[i][j];
        }
    }

    quaternionMin.Matrix(matrix3DMin);
    quaternionNext.Matrix(matrix3DNext);

    scanHMin = ignition::math::Angle(quaternionMin.Roll());
    ignition::math::Angle hStep((quaternionMin.Roll()-quaternionNext.Roll()));
    scanHMax = ignition::math::Angle(hStep.Radian() * scanHSamples);

    scanHMin.Normalize();
    scanHMax.Normalize();

    outMsg.set_angle_min(-scanHMin.Radian());
    outMsg.set_angle_max(-scanHMax.Radian());

    outMsg.set_angle_step(hStep.Radian());

    if (outMsg.ranges_size() != hitpointCount) {
        for (int i=0; i < hitpointCount; ++i) {
            outMsg.add_ranges(resultDistances[i]);
            outMsg.add_intensities(100.0);
        }
    }

    return outMsg;
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
    if (!publishLaserScan) {
        pointCloudPublisher = ignition::transport::Node::Publisher();
    } else {
        laserScanPublisher = ignition::transport::Node::Publisher();
    }
    pointCloudWorldPublisher = ignition::transport::Node::Publisher();
    isLidarInitialized = false;
}

}  // namespace rgl
