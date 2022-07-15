#pragma once

#include <ignition/common/MeshManager.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <set>
#include <ignition/gazebo/components/Geometry.hh>

namespace rgl {
    class RGLGazeboPlugin :
            public ignition::gazebo::System,
            public ignition::gazebo::ISystemPreUpdate,
            public ignition::gazebo::ISystemPostUpdate {

    public:
        RGLGazeboPlugin();

        ~RGLGazeboPlugin() override;

        void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

        void PostUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    private:
        std::chrono::steady_clock::duration sim_time{0};

        std::set<ignition::gazebo::Entity> entities_in_rgl;

        ignition::gazebo::Entity lidar{0};

        ignition::common::MeshManager* mesh_manager{ignition::common::MeshManager::Instance()};

        static ignition::math::Pose3<double> FindWorldPose(const ignition::gazebo::Entity &_entity,
                                                           const ignition::gazebo::EntityComponentManager &_ecm);

        bool EntityInRGL(ignition::gazebo::Entity entity);

        bool GetMesh(const ignition::gazebo::components::Geometry* geometry, unsigned int* v_count,
                     unsigned int* i_count, float** vertices, int** indices);
    };
}