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

#include "RGLServerPluginManager.hh"

#include <ignition/gazebo/components/Pose.hh>

#define ROUND_BY_VALUE 10000

using namespace rgl;

ignition::math::Pose3<double> RGLServerPluginManager::FindWorldPose(
        const ignition::gazebo::Entity& entity,
        const ignition::gazebo::EntityComponentManager& ecm) {

    auto local_pose = ecm.Component<ignition::gazebo::components::Pose>(entity);
    if (nullptr == local_pose) {
        ignmsg << "pose data missing - using default pose (0, 0, 0, 0, 0, 0)\n";
        return ignition::math::Pose3d::Zero;
    }
    auto world_pose = local_pose->Data();

    ignition::gazebo::Entity this_entity = entity;
    ignition::gazebo::Entity parent;

    while ((parent = ecm.ParentEntity(this_entity)) != WORLD_ENTITY_ID) {
        auto parent_pose = ecm.Component<ignition::gazebo::components::Pose>(parent);
        if (nullptr == parent_pose) {
            ignmsg << "pose data missing - using default pose (0, 0, 0, 0, 0, 0)\n";
            return ignition::math::Pose3d::Zero;
        }
        world_pose += parent_pose->Data();
        this_entity = parent;
    }

    return world_pose;
}

rgl_mat3x4f RGLServerPluginManager::GetRglMatrix(
        ignition::gazebo::Entity entity,
        const ignition::gazebo::EntityComponentManager& ecm) {

    auto gazebo_matrix = ignition::math::Matrix4<double>(FindWorldPose(entity, ecm));

    rgl_mat3x4f rgl_matrix;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            rgl_matrix.value[i][j] = static_cast<float>(gazebo_matrix(i, j));
        }
    }

    return rgl_matrix;
}


float RGLServerPluginManager::RoundFloat(float value) {
    return std::roundf(value * ROUND_BY_VALUE) / ROUND_BY_VALUE;
}
