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

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Pose.hh>

namespace rgl
{

// Checks RGL native call output status.
// Returns true when success.
// When API call returns error status, it prints error string and returns false.
bool CheckRGL(rgl_status_t status);

ignition::math::Pose3<double> FindWorldPose(
        const ignition::gazebo::Entity& entity,
        const ignition::gazebo::EntityComponentManager& ecm);

rgl_mat3x4f FindWorldPoseInRglMatrix(
        const ignition::gazebo::Entity& entity,
        const ignition::gazebo::EntityComponentManager& ecm);

rgl_mat3x4f IgnPose3dToRglMatrix(const ignition::math::Pose3<double>& pose);


// Throws exception when version of RGL library mismatch
void ValidateRGLVersion();

}  // namespace rgl
