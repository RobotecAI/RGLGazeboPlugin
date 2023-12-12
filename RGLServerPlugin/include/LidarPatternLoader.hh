// Copyright 2023 Robotec.AI
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
#include <vector>

#include <rgl/api/core.h>

#include <gz/sim/System.hh>

namespace rgl
{

class LidarPatternLoader
{
public:
    using LoadFuncType = std::function<bool(const sdf::ElementConstPtr&, std::vector<rgl_mat3x4f>&)>;

    static bool Load(const sdf::ElementConstPtr& sdf, std::vector<rgl_mat3x4f>& outPattern);

private:
    LidarPatternLoader() {};

    static bool LoadAnglesAndSamplesElement(const sdf::ElementConstPtr& sdf,
                                            gz::math::Angle& angleMin, gz::math::Angle& angleMax,
                                            int& samples);

    static bool LoadPatternFromUniform(const sdf::ElementConstPtr& sdf, std::vector<rgl_mat3x4f>& outPattern);
    static bool LoadPatternFromCustom(const sdf::ElementConstPtr& sdf, std::vector<rgl_mat3x4f>& outPattern);
    static bool LoadPatternFromPreset(const sdf::ElementConstPtr& sdf, std::vector<rgl_mat3x4f>& outPattern);
    static bool LoadPatternFromPresetPath(const sdf::ElementConstPtr& sdf, std::vector<rgl_mat3x4f>& outPattern);

    static rgl_mat3x4f AnglesToRglMat3x4f(const gz::math::Angle& roll,
                                          const gz::math::Angle& pitch,
                                          const gz::math::Angle& yaw);
    
    template<typename T>
    static std::vector<T> LoadVector(const std::filesystem::path& path);

    static std::map<std::string, std::string> presetNameToFilename;
    static std::map<std::string, LoadFuncType> patternLoadFunctions;
};

}  // namespace rgl
