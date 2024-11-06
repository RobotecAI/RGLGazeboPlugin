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

#include <cstdlib>
#include <tuple>

#include "LidarPatternLoader.hh"
#include "gz/math/Matrix4.hh"

#define PATTERNS_DIR_ENV "RGL_PATTERNS_DIR"

using namespace std::placeholders;
namespace fs = std::filesystem;

namespace rgl
{

// map preset name to pair of filename and sample size (for alternating patterns)
// set sample size to 0 to indicate non-alternating patterns
std::map<std::string, std::pair<std::string, std::size_t>> LidarPatternLoader::presetNameToFilename = {
    {"Alpha Prime", {"VelodyneVLS128.mat3x4f", 0}},
    {"Puck", {"VelodyneVLP16.mat3x4f", 0}},
    {"Ultra Puck", {"VelodyneVLP32C.mat3x4f", 0}},
    {"OS1 64", {"OusterOS1_64.mat3x4f", 0}},
    {"Pandar64", {"HesaiPandarQT64.mat3x4f", 0}},
    {"Pandar40P", {"HesaiPandar40P.mat3x4f", 0}},
    {"Livox Mid360", {"LivoxMid360.mat3x4f", 20000}},
};

std::map<std::string, LidarPatternLoader::LoadFuncType> LidarPatternLoader::patternLoadFunctions = {
    {"pattern_uniform", std::bind(&LidarPatternLoader::LoadPatternFromUniform, _1, _2, _3)},
    {"pattern_custom", std::bind(&LidarPatternLoader::LoadPatternFromCustom, _1, _2, _3)},
    {"pattern_preset", std::bind(&LidarPatternLoader::LoadPatternFromPreset, _1, _2, _3)},
    {"pattern_preset_path", std::bind(&LidarPatternLoader::LoadPatternFromPresetPath, _1, _2, _3)},
    {"pattern_lidar2d", std::bind(&LidarPatternLoader::LoadPatternFromLidar2d, _1, _2, _3)},
};

bool LidarPatternLoader::Load(const sdf::ElementConstPtr& sdf, std::vector<rgl_mat3x4f>& outPattern,
                              std::size_t& outPatternSampleSize)
{
    for (const auto &[patterName, loadFunction]: patternLoadFunctions)
    {
        if (!sdf->HasElement(patterName)) {
            continue;
        }
        gzmsg << "Trying to load '" << patterName << "' pattern...\n";
        if (loadFunction(sdf->FindElement(patterName), outPattern, outPatternSampleSize)) {
            gzmsg << "Successfully loaded pattern '" << patterName << "'.\n";
            return true;
        }
    }
    gzerr << "Failed to load lidar pattern. See plugin's documentation for available options.\n";
    return false;
}

bool LidarPatternLoader::LoadAnglesAndSamplesElement(const sdf::ElementConstPtr& sdf,
                                                     gz::math::Angle& angleMin, gz::math::Angle& angleMax,
                                                     int& samples)
{
    if (!sdf->HasElement("samples")) {
        gzerr << "Failed to load '" << sdf->GetName() << "' element. A 'samples' element is required inside, but it is not set.\n";
        return false;
    }

    if (!sdf->HasElement("min_angle")) {
        gzerr << "Failed to load '" << sdf->GetName() << "' element. A 'min_angle' element is required inside, but it is not set.\n";
        return false;
    }

    if (!sdf->HasElement("max_angle")) {
        gzerr << "Failed to load '" << sdf->GetName() << "' element. A 'max_angle' element is required inside, but it is not set.\n";
        return false;
    }

    angleMin = sdf->Get<float>("min_angle");
    angleMax = sdf->Get<float>("max_angle");
    samples = sdf->Get<int>("samples");

    if (angleMin > angleMax) {
        gzerr << "Failed to load '" << sdf->GetName() << "' element. Min angle greater than vertical max angle.\n";
        return false;
    }

    if (samples <= 0) {
        gzerr << "Failed to load '" << sdf->GetName() << "' element. Samples must be a positive value.\n";
        return false;
    }

    return true;
}

bool LidarPatternLoader::LoadPatternFromUniform(const sdf::ElementConstPtr& sdf, std::vector<rgl_mat3x4f>& outPattern, std::size_t& outPatternSampleSize)
{
    if (!sdf->HasElement("vertical")) {
        gzerr << "Failed to load uniform pattern. A vertical element is required, but it is not set.\n";
        return false;
    }

    if (!sdf->HasElement("horizontal")) {
        gzerr << "Failed to load uniform pattern. A horizontal element is required, but it is not set.\n";
        return false;
    }

    gz::math::Angle vMin, vMax, hMin, hMax;
    int vSamples, hSamples;

    if (!LoadAnglesAndSamplesElement(sdf->FindElement("vertical"), vMin, vMax, vSamples)) {
        return false;
    }

    if (!LoadAnglesAndSamplesElement(sdf->FindElement("horizontal"), hMin, hMax, hSamples)) {
        return false;
    }

    outPattern.reserve(vSamples * hSamples);

    gz::math::Angle vStep((vMax - vMin) / static_cast<double>(vSamples));
    gz::math::Angle hStep((hMax - hMin) / static_cast<double>(hSamples));

    auto vAngle = vMin;
    for (int i = 0; i < vSamples; ++i) {
        auto hAngle = hMin;
        for (int j = 0; j < hSamples; ++j) {
            outPattern.push_back(
                AnglesToRglMat3x4f(gz::math::Angle::Zero,
                                   // Inverse and shift 90deg pitch to match uniform pattern from Gazebo
                                   vAngle * -1 + gz::math::Angle::HalfPi,
                                   hAngle));
            hAngle += hStep;
        }
        vAngle += vStep;
    }

    outPatternSampleSize = outPattern.size();

    return true;
}

bool LidarPatternLoader::LoadPatternFromCustom(const sdf::ElementConstPtr& sdf, std::vector<rgl_mat3x4f>& outPattern, std::size_t& outPatternSampleSize)
{
    if (!sdf->HasAttribute("channels")) {
        gzerr << "Failed to load custom pattern. A channels attribute is required, but it is not set.\n";
        return false;
    }

    auto channelAngles = sdf->GetAttribute("channels");

    std::vector<gz::math::Angle> channels;
    std::istringstream iss(channelAngles->GetAsString());
    std::copy(std::istream_iterator<gz::math::Angle>(iss),
              std::istream_iterator<gz::math::Angle>(),
              std::back_inserter(channels));

    if (channels.empty()) {
        gzerr << "Failed to load custom pattern. No channels provided.\n";
        return false;
    }

    gz::math::Angle hMin, hMax;
    int hSamples;
    if (!LoadAnglesAndSamplesElement(sdf->FindElement("horizontal"), hMin, hMax, hSamples)) {
        return false;
    }

    gz::math::Angle hStep((hMax - hMin) / static_cast<double>(hSamples));

    outPattern.reserve(channels.size() * hSamples);

    for (auto channel : channels) {
        auto hAngle = hMin;
        for (int j = 0; j < hSamples; ++j) {
            outPattern.push_back(
                AnglesToRglMat3x4f(gz::math::Angle::Zero,
                                   // Inverse and shift 90deg pitch to match uniform pattern from Gazebo
                                   channel * -1 + gz::math::Angle::HalfPi,
                                   hAngle));
            hAngle += hStep;
        }
    }

    outPatternSampleSize = outPattern.size();

    return true;
}

bool LidarPatternLoader::LoadPatternFromPreset(const sdf::ElementConstPtr& sdf, std::vector<rgl_mat3x4f>& outPattern, std::size_t& outPatternSampleSize)
{
    auto presetName = sdf->Get<std::string>();
    if (!presetNameToFilename.contains(presetName)) {
        gzerr << "Failed to load preset pattern. Preset '" << presetName << "' is not available.\n";
        return false;
    }
    fs::path presetPath;
    std::tie(presetPath, outPatternSampleSize) = presetNameToFilename[presetName];
    if (const char* presetDir = std::getenv(PATTERNS_DIR_ENV)) {
        presetPath = fs::path(presetDir) / presetPath;
    }
    gzmsg << "Loading pattern_preset '" << presetName << "'...\n";
    outPattern = LoadVector<rgl_mat3x4f>(presetPath);
    if (outPattern.size() == 0) {
        gzerr << "Failed to load preset. Make sure the environment variable '" << PATTERNS_DIR_ENV << "' is set correctly.\n";
        return false;
    }

    if (outPatternSampleSize == 0) {
        // non-alternating patterns have sample size equal to total pattern size
        outPatternSampleSize = outPattern.size();
    }
    else if (outPattern.size() % outPatternSampleSize != 0) {
        gzerr << "Failed to load preset alternating pattern. Total pattern size (" << outPattern.size() << ") must be a multiple of the sample size!\n";
        return false;
    }

    return true;
}

bool LidarPatternLoader::LoadPatternFromPresetPath(const sdf::ElementConstPtr& sdf, std::vector<rgl_mat3x4f>& outPattern, std::size_t& outPatternSampleSize)
{
    fs::path presetPath = fs::path(sdf->Get<std::string>());
    gzmsg << "Loading preset from path '" << presetPath << "'...\n";
    outPattern = LoadVector<rgl_mat3x4f>(presetPath);
    if (outPattern.size() == 0) {
        gzerr << "Failed to load preset from path.\n";
        return false;
    }

    outPatternSampleSize = outPattern.size();

    return true;
}

bool LidarPatternLoader::LoadPatternFromLidar2d(const sdf::ElementConstPtr& sdf, std::vector<rgl_mat3x4f>& outPattern, std::size_t& outPatternSampleSize)
{
    if (!sdf->HasElement("horizontal")) {
        gzerr << "Failed to load uniform pattern. A horizontal element is required, but it is not set.\n";
        return false;
    }

    gz::math::Angle hMin, hMax;
    int hSamples;

    if (!LoadAnglesAndSamplesElement(sdf->FindElement("horizontal"), hMin, hMax, hSamples)) {
        return false;
    }

    outPattern.clear();
    outPattern.reserve(hSamples);

    gz::math::Angle hStep((hMax - hMin) / static_cast<double>(hSamples));

    auto hAngle = hMin;
    for (int i = 0; i < hSamples; ++i) {
        outPattern.push_back(
            AnglesToRglMat3x4f(gz::math::Angle::Zero,
                                // Inverse and shift 90deg pitch to match uniform pattern from Gazebo
                                gz::math::Angle::HalfPi,
                                hAngle));
        hAngle += hStep;
    }

    outPatternSampleSize = outPattern.size();

    return true;
}

template<typename T>
std::vector<T> LidarPatternLoader::LoadVector(const fs::path& path)
{
    // open the file:
    std::streampos fileSize;
    std::ifstream file(path, std::ios::binary);

    if (!file.is_open() || file.eof()) {
       gzerr << "failed to open file '" << path << "' or file is empty, data will not be loaded.\n";
       return std::vector<T>();
    }

    // get its size:
    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    if (fileSize % sizeof(T) != 0) {
        gzerr << "invalid file size: '" << path << "'.\n";
        return std::vector<T>();
    }

    // read the data:
    std::vector<T> fileData(fileSize / sizeof(T));
    file.read((char*) &fileData[0], fileSize);
    return fileData;
}

rgl_mat3x4f LidarPatternLoader::AnglesToRglMat3x4f(const gz::math::Angle& roll,
                                                   const gz::math::Angle& pitch,
                                                   const gz::math::Angle& yaw)
{
    gz::math::Quaterniond quaternion(roll.Radian(), pitch.Radian(), yaw.Radian());
    gz::math::Matrix4d matrix4D(quaternion);

    rgl_mat3x4f rglMatrix;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            rglMatrix.value[i][j] = static_cast<float>(matrix4D(i, j));
        }
    }
    return rglMatrix;
}

}  // namespace rgl
