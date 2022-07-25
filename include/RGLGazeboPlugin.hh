#pragma once

#include <ignition/common/MeshManager.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <unordered_map>
#include <ignition/gazebo/components/Geometry.hh>
#include <rgl/api/experimental.h>
#include <rgl/api/e2e_extensions.h>

namespace rgl {
    class RGLGazeboPlugin :
            public ignition::gazebo::System,
            public ignition::gazebo::ISystemConfigure,
            public ignition::gazebo::ISystemPreUpdate,
            public ignition::gazebo::ISystemPostUpdate {

    public:
        RGLGazeboPlugin();

        ~RGLGazeboPlugin() override;

        void Configure(const ignition::gazebo::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       ignition::gazebo::EntityComponentManager &_ecm,
                       ignition::gazebo::EventManager &_eventMgr) override;

        void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

        void PostUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    private:
        std::unordered_map<ignition::gazebo::Entity, std::pair<rgl_entity_t ,rgl_mesh_t>> entities_in_rgl;

        ignition::gazebo::Entity gazebo_lidar{8};

        rgl_lidar_t rgl_lidar{0};

        ignition::common::MeshManager* mesh_manager{ignition::common::MeshManager::Instance()};

        static ignition::math::Pose3<double> FindWorldPose(const ignition::gazebo::Entity &_entity,
                                                           const ignition::gazebo::EntityComponentManager &_ecm);

        bool EntityInRGL(ignition::gazebo::Entity entity);

        bool GetMesh(const ignition::gazebo::components::Geometry* geometry, int& v_count,
                     int& i_count, rgl_vec3f*& vertices, rgl_vec3i** indices);

        void UpdateRGLEntitiesPose(const ignition::gazebo::EntityComponentManager &_ecm);

        void UpdateLidarPose(const ignition::gazebo::EntityComponentManager &_ecm);

        void CreateLidar(ignition::gazebo::EntityComponentManager &_ecm);

        rgl_mat3x4f GetRglMatrix(ignition::gazebo::Entity entity, const ignition::gazebo::EntityComponentManager &_ecm);

        bool LoadMeshToRGL(rgl_mesh_t* new_mesh, const ignition::gazebo::components::Geometry* geometry);
    };
}