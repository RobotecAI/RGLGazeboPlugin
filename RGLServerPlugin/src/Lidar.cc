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

#define RGL_POINT_CLOUD_TEXT "/RGL_point_cloud_"

#define ONE_THOUSAND 1000
#define ONE_MILLION 1000000

#define UPDATE_RATE_TEXT "update_rate"
#define RANGE_TEXT "range"
#define ALWAYS_ON_TEXT "always_on"
#define CUSTOM_PATTERN_TEXT "custom_pattern"
#define HORIZONTAL_TEXT "horizontal"
#define VERTICAL_TEXT "vertical"
#define MAX_ANGLE_TEXT "max_angle"
#define MIN_ANGLE_TEXT "min_angle"
#define SAMPLES_TEXT "samples"
#define PATTERN_TEXT "pattern"

#define ALPHA_PRIME_TEXT "Alpha Prime"

using namespace rgl;

void RGLServerPluginInstance::LoadConfiguration(const std::shared_ptr<const sdf::Element>& sdf) {

    if (sdf->FindElement(UPDATE_RATE_TEXT) != nullptr) {
        int64_t update_rate = sdf->Get<int>(UPDATE_RATE_TEXT);
        time_between_raytraces = std::chrono::microseconds (static_cast<int64_t>((ONE_MILLION / update_rate)));
        updates_between_raytraces = static_cast<int>((ONE_THOUSAND / update_rate));
        ignmsg << "update rate specified: " << update_rate << " Hz\n";
    } else {
        ignmsg << "no update rate specified, using default update rate of 10 Hz\n";
    }

    if (sdf->FindElement(RANGE_TEXT) != nullptr) {
        range = sdf->Get<float>(RANGE_TEXT);
        ignmsg << "raytrace range specified: " << range << " units\n";
    } else {
        ignmsg << "no range specified, using default range of " << range << " units\n";
    }

    if (sdf->FindElement(ALWAYS_ON_TEXT) != nullptr) {
        always_on = sdf->Get<bool>(ALWAYS_ON_TEXT);
        if (always_on) {
            ignmsg << "RGL will be active when simulation is paused\n";
        } else {
            ignmsg << "RGL will NOT be active when simulation is paused\n";
        }
    } else {
        ignmsg << "not specified, whether RGL should be active in paused simulation state, "
               << "it is NOT active in paused simulation state by default\n";
    }

    if (sdf->FindElement(CUSTOM_PATTERN_TEXT) != nullptr) {
        ignmsg << "custom pattern selected\n";

        auto custom_pattern = sdf->FindElement(CUSTOM_PATTERN_TEXT);

        if (custom_pattern->FindElement(HORIZONTAL_TEXT) != nullptr) {
            auto horizontal = custom_pattern->FindElement(HORIZONTAL_TEXT);
            if (horizontal->FindElement(SAMPLES_TEXT) != nullptr) {
                samples_horizontal = horizontal->Get<int>(SAMPLES_TEXT);
                ignmsg << "number of horizontal samples specified: " << samples_horizontal << "\n";
            } else {
                ignmsg << "number of horizontal samples NOT specified, using default value: " << samples_horizontal << "\n";
            }
            if (horizontal->FindElement(MIN_ANGLE_TEXT) != nullptr) {
                horizontal_min = horizontal->Get<float>(MIN_ANGLE_TEXT);
                if (horizontal_min < ignition::math::Angle::Pi * -1) {
                    horizontal_min = ignition::math::Angle::Pi * -1;
                    ignwarn << "horizontal min angle can NOT have a value lesser than -Pi, using -Pi\n";
                } else {
                    ignmsg << "horizontal min angle specified: " << horizontal_min << " radians\n";
                }
            } else {
                ignmsg << "horizontal min angle NOT specified, using default value: " << horizontal_min << " radians\n";
            }
            if (horizontal->FindElement(MAX_ANGLE_TEXT) != nullptr) {
                horizontal_max = horizontal->Get<float>(MAX_ANGLE_TEXT);
                if (horizontal_max > ignition::math::Angle::Pi) {
                    horizontal_max = ignition::math::Angle::Pi;
                    ignwarn << "horizontal max angle can NOT have a value greater than Pi, using Pi\n";
                } else {
                    ignmsg << "horizontal max angle specified: " << horizontal_max << " radians\n";
                }
            } else {
                ignmsg << "horizontal max angle NOT specified, using default value: " << horizontal_max << " radians\n";
            }
            if (horizontal_min > horizontal_max) {
                horizontal_min = ignition::math::Angle::Pi * -1;
                horizontal_max = ignition::math::Angle::Pi;
                ignwarn << "horizontal min angle greater than horizontal max angle, using defaults for both angles: "
                << "horizontal min: " << horizontal_min << " radians, and horizontal max: " << horizontal_max << " radians\n";
            }
        } else {
            ignmsg << "no horizontal fov specified, using default " << horizontal_max - horizontal_min << " radians\n";
        }

        if (custom_pattern->FindElement(VERTICAL_TEXT) != nullptr) {
            auto vertical = custom_pattern->FindElement(VERTICAL_TEXT);
            if (vertical->FindElement(SAMPLES_TEXT) != nullptr) {
                samples_vertical = vertical->Get<int>(SAMPLES_TEXT);
                ignmsg << "number of vertical samples specified: " << samples_vertical << "\n";
            } else {
                ignmsg << "number of vertical samples NOT specified, using default value: " << samples_vertical << "\n";
            }
            if (vertical->FindElement(MIN_ANGLE_TEXT) != nullptr) {
                vertical_min = vertical->Get<float>(MIN_ANGLE_TEXT);
                if (vertical_min < ignition::math::Angle::HalfPi * -1) {
                    vertical_min = ignition::math::Angle::HalfPi * -1;
                    ignwarn << "vertical min angle can NOT have a value lesser than -Pi / 2, using -Pi / 2\n";
                } else {
                    ignmsg << "vertical min angle specified: " << vertical_min << " radians\n";
                }
            } else {
                ignmsg << "vertical min angle NOT specified, using default value: " << vertical_min << " radians\n";
            }
            if (vertical->FindElement(MAX_ANGLE_TEXT) != nullptr) {
                vertical_max = vertical->Get<float>(MAX_ANGLE_TEXT);
                if (vertical_max > ignition::math::Angle::HalfPi) {
                    vertical_max = ignition::math::Angle::HalfPi;
                    ignwarn << "vertical max angle can NOT have a value greater than Pi / 2, using Pi / 2\n";
                } else {
                    ignmsg << "vertical max angle specified: " << vertical_max << " radians\n";
                }
            } else {
                ignmsg << "vertical max angle NOT specified, using default value: " << vertical_max << " radians\n";
            }
            if (vertical_min > vertical_max) {
                vertical_min = ignition::math::Angle::HalfPi * -1;
                vertical_max = ignition::math::Angle::HalfPi;
                ignwarn << "vertical min angle greater than vertical max angle, using defaults for both angles: "
                        << "vertical min: " << vertical_min << " radians, and vertical max: " << vertical_max << " radians\n";
            }
        } else {
            ignmsg << "no vertical fov specified, using default " << vertical_max - vertical_min << " radians\n";
        }

    } else if (sdf->FindElement(PATTERN_TEXT) != nullptr) {
        if (sdf->Get<std::string>(PATTERN_TEXT) == ALPHA_PRIME_TEXT) {
            samples_horizontal = 3600;
            horizontal_min.SetDegree(-180.0);
            horizontal_max.SetDegree(180.0);
            samples_vertical = 128;
            vertical_min.SetDegree(-25.0);
            vertical_max.SetDegree(15.0);
            ignmsg << "using Alpha Prime pattern\n";
        } else {
            //TODO: implement when patterns will be supported in RGL
            //pattern_name = sdf->Get<std::string>(PATTERN_TEXT);
            ignmsg << "patterns not implemented yet, using default pattern with " << samples_vertical << " samples_vertical and "
                   << samples_horizontal << " rays each, " << "vertical fov of " << vertical_max - vertical_min
                   << " radians and horizontal fov of " << horizontal_max - horizontal_min << " radians. "
                   << "To see what lidar patterns are supported, visit https://github.com/RobotecAI/RobotecGPULidar\n";
        }
    } else {
        ignmsg << "pattern not specified, using default pattern with " << samples_vertical << " samples_vertical and "
               << samples_horizontal << " rays each, " << "vertical fov of " << vertical_max - vertical_min
               << " radians and horizontal fov of " << horizontal_max - horizontal_min << " radians. "
               << "To see what lidar patterns are supported, visit https://github.com/RobotecAI/RobotecGPULidar\n";
    }
}

void RGLServerPluginInstance::AddToRayTf(std::vector<rgl_mat3x4f>& ray_tf,
                const ignition::math::Angle& roll,
                const ignition::math::Angle& pitch,
                const ignition::math::Angle& jaw) {

    ignition::math::Quaterniond quaternion(roll.Radian(), pitch.Radian(), jaw.Radian());
    ignition::math::Matrix4d matrix4D(quaternion);
    rgl_mat3x4f rgl_matrix;
    for (int l = 0; l < 3; ++l) {
        for (int m = 0; m < 4; ++m) {
            rgl_matrix.value[l][m] = static_cast<float>(matrix4D(l, m));
        }
    }
    ray_tf.push_back(rgl_matrix);
}

void RGLServerPluginInstance::CreateLidar(ignition::gazebo::Entity entity,
                                          ignition::gazebo::EntityComponentManager& ecm) {

    lidar_exists = true;
    gazebo_lidar = entity;
    static int next_free_id = 0;
    lidar_id = next_free_id;
    next_free_id++;

    int rays = samples_vertical * samples_horizontal;
    std::vector<rgl_mat3x4f> ray_tf;

    ignition::math::Angle vertical_step((vertical_max - vertical_min) / static_cast<double>(samples_vertical));
    ignition::math::Angle horizontal_step((horizontal_max - horizontal_min) / static_cast<double>(samples_horizontal));

    ignition::math::Angle pitch(ignition::math::Angle::HalfPi - vertical_max);
    ignition::math::Angle jaw(horizontal_min);

    for (int i = 0; i < samples_vertical; ++i, pitch += vertical_step) {
        for (int j = 0; j < samples_horizontal; ++j, jaw += horizontal_step) {
            AddToRayTf(ray_tf, ignition::math::Angle::Zero, pitch, jaw);
        }
    }

    auto rgl_pose_matrix = RGLServerPluginManager::GetRglMatrix(gazebo_lidar, ecm);

    RGL_CHECK(rgl_node_rays_from_mat3x4f(&node_use_rays, ray_tf.data(), rays));
    RGL_CHECK(rgl_node_rays_transform(&node_lidar_pose, &rgl_pose_matrix));
    RGL_CHECK(rgl_node_raytrace(&node_raytrace, nullptr, range));
    RGL_CHECK(rgl_node_points_compact(&node_compact));

    RGL_CHECK(rgl_graph_node_add_child(node_use_rays, node_lidar_pose));
    RGL_CHECK(rgl_graph_node_add_child(node_lidar_pose, node_raytrace));
    RGL_CHECK(rgl_graph_node_add_child(node_raytrace, node_compact));

    pointcloud_publisher = node.Advertise<ignition::msgs::PointCloudPacked>(
             RGL_POINT_CLOUD_TEXT + std::to_string(lidar_id));
}

void RGLServerPluginInstance::UpdateLidarPose(const ignition::gazebo::EntityComponentManager& ecm) {

    auto rgl_pose_matrix = RGLServerPluginManager::GetRglMatrix(gazebo_lidar, ecm);
    RGL_CHECK(rgl_node_rays_transform(&node_lidar_pose, &rgl_pose_matrix));
}

bool RGLServerPluginInstance::ShouldRayTrace(std::chrono::steady_clock::duration sim_time,
                                             bool paused) {

    if (!lidar_exists) {
        return false;
    }
    if (always_on) {
        if (before_first_raytrace) {
            before_first_raytrace = false;
            return true;
        }
        if (!paused && sim_time < last_raytrace_time + time_between_raytraces) {
            return false;
        }
        if (paused && current_update < last_raytrace_update + updates_between_raytraces) {
            return false;
        }
    } else{
        if (before_first_raytrace && !paused) {
            before_first_raytrace = false;
            return true;
        }
        if (sim_time < last_raytrace_time + time_between_raytraces) {
            return false;
        }
    }
    return true;
}

void RGLServerPluginInstance::RayTrace(std::chrono::steady_clock::duration sim_time) {

    last_raytrace_time = sim_time;
    last_raytrace_update = current_update;

    //TODO: remove this when rgl is fixed
    if (current_update == 1) {
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

    ignition::msgs::PointCloudPacked point_cloud_msg;
    ignition::msgs::InitPointCloudPacked(point_cloud_msg, "RGL", true,
                                         {{"xyz", ignition::msgs::PointCloudPacked::Field::FLOAT32}});
    point_cloud_msg.mutable_data()->resize(hitpoint_count * point_cloud_msg.point_step());
    point_cloud_msg.set_height(1);
    point_cloud_msg.set_width(hitpoint_count);

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
