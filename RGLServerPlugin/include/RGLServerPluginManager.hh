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

#include <ignition/common/MeshManager.hh>

#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/transport/Node.hh>

#define WORLD_ENTITY_ID 1

#define RGL_CHECK(call)                  \
do {                                     \
    rgl_status_t status = call;          \
    if (status != RGL_SUCCESS) {         \
        const char* msg;                 \
        rgl_get_last_error_string(&msg); \
        ignerr << msg;                   \
        exit(1);                         \
    }                                    \
} while(0)

namespace rgl {
    class RGLServerPluginManager :
            public ignition::gazebo::System,
            public ignition::gazebo::ISystemConfigure,
            public ignition::gazebo::ISystemPreUpdate,
            public ignition::gazebo::ISystemPostUpdate {

    public:
        RGLServerPluginManager();

        ~RGLServerPluginManager() override;

        // only called once, when plugin is being loaded
        void Configure(
                const ignition::gazebo::Entity& entity,
                const std::shared_ptr<const sdf::Element>& sdf,
                ignition::gazebo::EntityComponentManager& ecm,
                ignition::gazebo::EventManager& eventMgr) override;

        // called every time before physics update runs (can change entities)
        void PreUpdate(
                const ignition::gazebo::UpdateInfo& info,
                ignition::gazebo::EntityComponentManager& ecm) override;

        // called every time after physics runs (can't change entities)
        void PostUpdate(
                const ignition::gazebo::UpdateInfo& info,
                const ignition::gazebo::EntityComponentManager& ecm) override;

        ////////////////////////////// Utils /////////////////////////////////

        static ignition::math::Pose3<double> FindWorldPose(
                const ignition::gazebo::Entity& entity,
                const ignition::gazebo::EntityComponentManager& ecm);

        // get the local to global transform matrix
        static rgl_mat3x4f GetRglMatrix(ignition::gazebo::Entity entity,
                                        const ignition::gazebo::EntityComponentManager& ecm);

        // gazebo mesh factory had problems with adding multiple floats becoming unreliable
        static float RoundFloat(float value);

    private:
        ////////////////////////////////////////////// Variables /////////////////////////////////////////////
        ////////////////////////////// Lidar ////////////////////////////////
        // contains pointers to all entities that were loaded to rgl (as well as to their meshes)
        std::unordered_map<ignition::gazebo::Entity, std::pair<rgl_entity_t, rgl_mesh_t>> entities_in_rgl;

        // the entity ids, that the lidars are attached to
        std::unordered_set<ignition::gazebo::Entity> gazebo_lidars;

        // all entities, that the lidar should ignore
        std::unordered_set<ignition::gazebo::Entity> lidar_ignore;

        ////////////////////////////// Mesh /////////////////////////////////

        ignition::common::MeshManager* mesh_manager{ignition::common::MeshManager::Instance()};

        ////////////////////////////////////////////// Functions /////////////////////////////////////////////
        ////////////////////////////// Scene ////////////////////////////////

        bool CheckNewLidarsCb(
                ignition::gazebo::Entity entity,
                const ignition::gazebo::EntityComponentManager& ecm);

        bool CheckRemovedLidarsCb(
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

        void UpdateRGLEntityPose(const ignition::gazebo::EntityComponentManager& ecm);

        ////////////////////////////// Mesh /////////////////////////////////

        const ignition::common::Mesh* LoadBox(
                const sdf::Geometry& data,
                double& scale_x,
                double& scale_y,
                double& scale_z);

        const ignition::common::Mesh* LoadCapsule(
                const sdf::Geometry& data,
                double& scale_x,
                double& scale_y,
                double& scale_z);

        const ignition::common::Mesh* LoadCylinder(
                const sdf::Geometry& data,
                double& scale_x,
                double& scale_y,
                double& scale_z);

        const ignition::common::Mesh* LoadEllipsoid(
                const sdf::Geometry& data,
                double& scale_x,
                double& scale_y,
                double& scale_z);

        const ignition::common::Mesh* LoadMesh(
                const sdf::Geometry& data,
                double& scale_x,
                double& scale_y,
                double& scale_z);

        const ignition::common::Mesh* LoadPlane(
                const sdf::Geometry& data,
                double& scale_x,
                double& scale_y);

        const ignition::common::Mesh* LoadSphere(
                const sdf::Geometry& data,
                double& scale_x,
                double& scale_y,
                double& scale_z);

        // also gets the scale of the mesh
        const ignition::common::Mesh* GetMeshPointer(
                const sdf::Geometry& data,
                double& scale_x,
                double& scale_y,
                double& scale_z);

        bool LoadMeshToRGL(rgl_mesh_t* new_mesh, const sdf::Geometry& data);
    };
}
