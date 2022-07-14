#pragma once

#include <ignition/common/MeshManager.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <set>

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
        std::chrono::steady_clock::duration simTime{0};

        std::set<std::string> mesh_names;

        ignition::gazebo::Entity rgl_entity{0};

        ignition::common::MeshManager* meshManager{ignition::common::MeshManager::Instance()};

        static ignition::math::Pose3<double> FindWorldPose(const ignition::gazebo::Entity &_entity,
                                                           const ignition::gazebo::EntityComponentManager &_ecm);
    };
}