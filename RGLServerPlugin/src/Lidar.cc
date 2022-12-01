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
#include "RGLServerPluginManager.hh"

#define RAYS_IN_ONE_DIR 500
#define LIDAR_RANGE 1000

#define RGL_POINT_CLOUD_TEXT "/RGL_point_cloud_"

using namespace rgl;

void RGLServerPluginInstance::CreateLidar(ignition::gazebo::Entity entity) {
    lidar_exists = true;
    updates_between_raytraces = (int)std::chrono::duration_cast<std::chrono::milliseconds>(time_between_raytraces).count();
    gazebo_lidar = entity;
    static int next_free_id = 0;
    lidar_id = next_free_id;
    next_free_id++;

    int rays = RAYS_IN_ONE_DIR * RAYS_IN_ONE_DIR;
    std::vector<rgl_mat3x4f> ray_tf;
    ignition::math::Angle X;
    ignition::math::Angle Y;
    double unit_angle = ignition::math::Angle::TwoPi.Radian() / RAYS_IN_ONE_DIR;
    for (int i = 0; i < RAYS_IN_ONE_DIR; ++i, X.SetRadian(unit_angle * i)) {
        for (int j = 0; j < RAYS_IN_ONE_DIR; ++j, Y.SetRadian(unit_angle * j)) {
            ignition::math::Quaterniond quaternion(0, X.Radian(), Y.Radian());
            ignition::math::Matrix4d matrix4D(quaternion);
            rgl_mat3x4f rgl_matrix;
            for (int l = 0; l < 3; ++l) {
                for (int m = 0; m < 4; ++m) {
                    rgl_matrix.value[l][m] = static_cast<float>(matrix4D(l, m));
                }
            }
            ray_tf.push_back(rgl_matrix);
        }
    }

    rgl_mat3x4f identity = {
            .value = {
                    { 1, 0, 0, 0 },
                    { 0, 1, 0, 0 },
                    { 0, 0, 1, 0 },
            }
    };

    RGL_CHECK(rgl_node_rays_from_mat3x4f(&node_use_rays, ray_tf.data(), rays));
    RGL_CHECK(rgl_node_rays_transform(&node_lidar_pose, &identity));
    RGL_CHECK(rgl_node_raytrace(&node_raytrace, nullptr, LIDAR_RANGE));
    RGL_CHECK(rgl_node_points_compact(&node_compact));

    RGL_CHECK(rgl_graph_node_add_child(node_use_rays, node_lidar_pose));
    RGL_CHECK(rgl_graph_node_add_child(node_lidar_pose, node_raytrace));
    RGL_CHECK(rgl_graph_node_add_child(node_raytrace, node_compact));

    pointcloud_publisher = node.Advertise<ignition::msgs::PointCloudPacked>(
             RGL_POINT_CLOUD_TEXT + std::to_string(lidar_id));
}



void RGLServerPluginInstance::UpdateLidarPose(const ignition::gazebo::EntityComponentManager& ecm,
                                              std::chrono::steady_clock::duration sim_time,
                                              bool paused) {
    if (!lidar_exists) {
        return;
    }

    if (!paused && sim_time < last_raytrace_time + time_between_raytraces) {
        return;
    }

    if (!paused) {
        last_raytrace_time = sim_time;
    }

    if (paused && current_update < last_raytrace_update + updates_between_raytraces) {
        return;
    }
    last_raytrace_update = current_update;

    auto rgl_pose_matrix = RGLServerPluginManager::GetRglMatrix(gazebo_lidar, ecm);
    RGL_CHECK(rgl_node_rays_transform(&node_lidar_pose, &rgl_pose_matrix));
}

void RGLServerPluginInstance::RayTrace(std::chrono::steady_clock::duration sim_time,
                                       bool paused) {
    if (!lidar_exists) {
        return;
    }
    current_update++;

    if (current_update == 1) {
        return;
    }

    if (!paused && sim_time < last_raytrace_time + time_between_raytraces) {
        return;
    }

    if (paused && current_update < last_raytrace_update + updates_between_raytraces) {
        return;
    }

    RGL_CHECK(rgl_graph_run(node_compact));

    int hitpoint_count = 0;
    int size;
    RGL_CHECK(rgl_graph_get_result_size(node_compact, RGL_FIELD_XYZ_F32, &hitpoint_count, &size));

    if (size != sizeof(rgl_vec3f)) {
        ignerr << "invalid raytrace size of element: " << size << "\n";
        return;
    }

    if (hitpoint_count == 0) {
        return;
    }

    std::vector<rgl_vec3f> results(hitpoint_count, rgl_vec3f());
    RGL_CHECK(rgl_graph_get_result_data(node_compact, RGL_FIELD_XYZ_F32, results.data()));

    // Create message
    ignition::msgs::PointCloudPacked point_cloud_msg;
    ignition::msgs::InitPointCloudPacked(point_cloud_msg, "RGL", true,
                                         {{"xyz", ignition::msgs::PointCloudPacked::Field::FLOAT32}});
    point_cloud_msg.mutable_data()->resize(hitpoint_count * point_cloud_msg.point_step());
    point_cloud_msg.set_height(1);
    point_cloud_msg.set_width(hitpoint_count);

    // Populate message
    ignition::msgs::PointCloudPackedIterator<float> xIter(point_cloud_msg, "x");
    ignition::msgs::PointCloudPackedIterator<float> yIter(point_cloud_msg, "y");
    ignition::msgs::PointCloudPackedIterator<float> zIter(point_cloud_msg, "z");

    for (int i = 0; i < hitpoint_count; ++i, ++xIter, ++yIter, ++zIter)
    {
        *xIter = results[i].value[0];
        *yIter = results[i].value[1];
        *zIter = results[i].value[2];
    }

    pointcloud_publisher.Publish(point_cloud_msg);
}

bool RGLServerPluginInstance::CheckLidarExists(ignition::gazebo::Entity entity) {
    if (entity == gazebo_lidar) {
        lidar_exists = false;
        rgl_graph_destroy(node_compact);
        pointcloud_publisher = ignition::transport::Node::Publisher();
    }
    return true;
}
