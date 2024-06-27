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

#include <gz/common/MeshManager.hh>

#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/System.hh>

#include <gz/transport/Node.hh>

namespace rgl
{

class RGLServerPluginManager :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
    using MeshInfo = std::variant<std::monostate, const gz::common::Mesh*, gz::common::SubMesh>;

    RGLServerPluginManager() = default;
    ~RGLServerPluginManager() override = default;

    // only called once, when plugin is being loaded
    void Configure(
        const gz::sim::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        gz::sim::EntityComponentManager& ecm,
        gz::sim::EventManager& eventMgr) override;

    // called every time after physics runs (can't change entities)
    void PostUpdate(
        const gz::sim::UpdateInfo& info,
        const gz::sim::EntityComponentManager& ecm) override;

private:
    ////////////////////////////////////////////// Variables /////////////////////////////////////////////

    ////////////////////////////// Lidar ////////////////////////////////
    // contains pointers to all entities that were loaded to rgl (as well as to their meshes)
    std::unordered_map<gz::sim::Entity, std::pair<rgl_entity_t, rgl_mesh_t>> entitiesInRgl;

    // the entity ids, that the lidars are attached to
    std::unordered_set<gz::sim::Entity> lidarEntities;

    // all entities, that the lidar should ignore
    std::unordered_set<gz::sim::Entity> entitiesToIgnore;

    ////////////////////////////// Mesh /////////////////////////////////

    gz::common::MeshManager* meshManager{gz::common::MeshManager::Instance()};

    ////////////////////////////////////////////// Functions /////////////////////////////////////////////

    ////////////////////////////// Scene ////////////////////////////////

    bool RegisterNewLidarCb(
        gz::sim::Entity entity,
        const gz::sim::EntityComponentManager& ecm);

    bool UnregisterLidarCb(
        gz::sim::Entity entity,
        const gz::sim::EntityComponentManager& ecm);

    bool LoadEntityToRGLCb(
        const gz::sim::Entity& entity,
        const gz::sim::components::Visual*,
        const gz::sim::components::Geometry* geometry);

    bool RemoveEntityFromRGLCb(
        const gz::sim::Entity& entity,
        const gz::sim::components::Visual*,
        const gz::sim::components::Geometry*);

    void UpdateRGLEntityPoses(const gz::sim::EntityComponentManager& ecm);

    std::unordered_set<gz::sim::Entity> GetEntitiesInParentLink(
        gz::sim::Entity entity,
        const gz::sim::EntityComponentManager& ecm);

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
