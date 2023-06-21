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

#include <exception>

#include "Utils.hh"

#define WORLD_ENTITY_ID 1

namespace rgl
{

bool CheckRGL(rgl_status_t status)
{
    if (status == RGL_SUCCESS) {
        return true;
    }

    const char* msg;
    rgl_get_last_error_string(&msg);
    ignerr << msg << "\n";
    return false;
}

gz::math::Pose3<double> FindWorldPose(
        const gz::sim::Entity& entity,
        const gz::sim::EntityComponentManager& ecm)
{
    auto localPose = ecm.Component<gz::sim::components::Pose>(entity);
    if (nullptr == localPose) {
        ignmsg << "pose data missing - using default pose (0, 0, 0, 0, 0, 0)\n";
        return gz::math::Pose3d::Zero;
    }
    auto worldPose = localPose->Data();

    gz::sim::Entity thisEntity = entity;
    gz::sim::Entity parent;

    while ((parent = ecm.ParentEntity(thisEntity)) != WORLD_ENTITY_ID) {
        auto parentPose = ecm.Component<gz::sim::components::Pose>(parent);
        if (nullptr == parentPose) {
            ignmsg << "pose data missing - using default pose (0, 0, 0, 0, 0, 0)\n";
            return gz::math::Pose3d::Zero;
        }
        worldPose += parentPose->Data();
        thisEntity = parent;
    }

    return worldPose;
}

rgl_mat3x4f FindWorldPoseInRglMatrix(
        const gz::sim::Entity& entity,
        const gz::sim::EntityComponentManager& ecm)
{
    return IgnPose3dToRglMatrix(FindWorldPose(entity, ecm));
}

rgl_mat3x4f IgnPose3dToRglMatrix(
        const gz::math::Pose3<double>& pose)
{

    auto ignMatrix = gz::math::Matrix4<double>(pose);
    rgl_mat3x4f rglMatrix;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            rglMatrix.value[i][j] = static_cast<float>(ignMatrix(i, j));
        }
    }
    return rglMatrix;
}

void ValidateRGLVersion()
{
    int32_t outMajor, outMinor, outPatch;
    if (!CheckRGL(rgl_get_version_info(&outMajor, &outMinor, &outPatch))) {
        throw std::runtime_error("Failed to get RGL library version.");
    }

    if (outMajor != RGL_VERSION_MAJOR || outMinor != RGL_VERSION_MINOR || outPatch != RGL_VERSION_PATCH) {
        std::ostringstream oss;
        oss << "RGL library version: " << outMajor << "." << outMinor << "." << outPatch
            << " does not match RGL core.h version: " << RGL_VERSION_MAJOR << "." << RGL_VERSION_MINOR
            << "." << RGL_VERSION_PATCH << ".\n";
        throw std::runtime_error(oss.str());
    }
}

}  // namespace rgl
