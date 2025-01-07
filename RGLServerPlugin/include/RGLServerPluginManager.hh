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

#include "Utils.hh"

#include <ignition/common/MeshManager.hh>

#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/LaserRetro.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/transport/Node.hh>

namespace rgl
{

class RGLServerPluginManager :
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPostUpdate
{
public:
    using MeshInfo = std::variant<std::monostate, const ignition::common::Mesh*, ignition::common::SubMesh>;

    RGLServerPluginManager() = default;
    ~RGLServerPluginManager() override = default;

    // only called once, when plugin is being loaded
    void Configure(
        const ignition::gazebo::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        ignition::gazebo::EntityComponentManager& ecm,
        ignition::gazebo::EventManager& eventMgr) override;

    // called every time after physics runs (can't change entities)
    void PostUpdate(
        const ignition::gazebo::UpdateInfo& info,
        const ignition::gazebo::EntityComponentManager& ecm) override;

private:
    ////////////////////////////////////////////// Variables /////////////////////////////////////////////

    ////////////////////////////// Lidar ////////////////////////////////
    // contains pointers to all entities that were loaded to rgl (as well as to their meshes)
    std::unordered_map<ignition::gazebo::Entity, std::pair<rgl_entity_t, rgl_mesh_t>> entitiesInRgl;

    // whether to ignore entities attached to the same link as the lidar
    bool doIgnoreEntitiesInLidarLink{true};

    // the entity ids, that the lidars are attached to
    std::unordered_set<ignition::gazebo::Entity> lidarEntities;

    // all entities, that the lidar should ignore
    std::unordered_set<ignition::gazebo::Entity> entitiesToIgnore;

    ////////////////////////////// Mesh /////////////////////////////////

    ignition::common::MeshManager* meshManager{ignition::common::MeshManager::Instance()};

    ////////////////////////////////////////////// Functions /////////////////////////////////////////////

    ////////////////////////////// Scene ////////////////////////////////

    bool RegisterNewLidarCb(
        ignition::gazebo::Entity entity,
        const ignition::gazebo::EntityComponentManager& ecm);

    bool UnregisterLidarCb(
        ignition::gazebo::Entity entity,
        const ignition::gazebo::EntityComponentManager& ecm);

    bool LoadEntityToRGLCb(
        const ignition::gazebo::Entity& entity,
        const ignition::gazebo::components::Visual*,
        const ignition::gazebo::components::Geometry* geometry);

    bool RemoveEntityFromRGLCb(
        const ignition::gazebo::Entity& entity,
        const ignition::gazebo::components::Visual*,
        const ignition::gazebo::components::Geometry*);

    bool SetLaserRetroCb(
        const ignition::gazebo::Entity& entity,
        const ignition::gazebo::components::LaserRetro* laser_retro);

    void UpdateRGLEntityTransforms(const ignition::gazebo::EntityComponentManager& ecm);

    std::unordered_set<ignition::gazebo::Entity> GetEntitiesInParentLink(
        ignition::gazebo::Entity entity,
        const ignition::gazebo::EntityComponentManager& ecm);

    ////////////////////////////// Mesh /////////////////////////////////

    MeshInfo LoadBox(
        const sdf::Geometry& data,
        double& scaleX,
        double& scaleY,
        double& scaleZ);

    MeshInfo LoadCapsule(const sdf::Geometry& data);

    MeshInfo LoadCylinder(
        const sdf::Geometry& data,
        double& scaleX,
        double& scaleY,
        double& scaleZ);

    MeshInfo LoadEllipsoid(
        const sdf::Geometry& data,
        double& scaleX,
        double& scaleY,
        double& scaleZ);

    MeshInfo LoadMesh(
        const sdf::Geometry& data,
        double& scaleX,
        double& scaleY,
        double& scaleZ);

    MeshInfo LoadPlane(
        const sdf::Geometry& data,
        double& scaleX,
        double& scaleY);

    MeshInfo LoadSphere(
        const sdf::Geometry& data,
        double& scaleX,
        double& scaleY,
        double& scaleZ);

    // also gets the scale of the mesh
    MeshInfo GetMeshPointer(
        const sdf::Geometry& data,
        double& scaleX,
        double& scaleY,
        double& scaleZ);

    bool LoadMeshToRGL(rgl_mesh_t* mesh, const sdf::Geometry& data);
};

} // namespace rgl
