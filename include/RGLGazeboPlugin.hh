#pragma once

#include <unordered_map>
#include <ignition/common/MeshManager.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <rgl/api/experimental.h>

#define RGL_CHECK(call)                  \
do {                                     \
    rgl_status_t status = call;          \
    if (status != RGL_SUCCESS) {         \
        const char* msg;                 \
        rgl_get_last_error_string(&msg); \
        ignmsg << msg;                   \
        exit(1);                         \
    }                                    \
} while(0)

namespace rgl {
    class RGLGazeboPlugin :
            public ignition::gazebo::System,
            public ignition::gazebo::ISystemConfigure,
            public ignition::gazebo::ISystemPreUpdate,
            public ignition::gazebo::ISystemPostUpdate {

    public:
        RGLGazeboPlugin();

        ~RGLGazeboPlugin() override;

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

    private:
        ////////////////////////////////////////////// Variables /////////////////////////////////////////////
        ////////////////////////////// Lidar ////////////////////////////////
        // contains pointers to all entities that were loaded to rgl (as well as to their meshes)
        std::unordered_map<ignition::gazebo::Entity, std::pair<rgl_entity_t, rgl_mesh_t>> entities_in_rgl;

        // the entity id, that the lidar is attached to (is set in Configure function)
        ignition::gazebo::Entity gazebo_lidar{0};

        // lidar entity from rgl
        rgl_lidar_t rgl_lidar{0};

        bool lidar_created{false};

        // entity representing lidar output
        ignition::gazebo::Entity rgl_visual{0};

        int lidar_id{-1};

        // all entities, that the lidar should ignore
        std::set<ignition::gazebo::Entity> lidar_ignore;

        // time of last RayTrace call
        std::chrono::steady_clock::duration last_update{0};

        bool ray_trace{false};

        ////////////////////////////// Mesh /////////////////////////////////

        ignition::common::MeshManager* mesh_manager{ignition::common::MeshManager::Instance()};

        ////////////////////////////// Utils /////////////////////////////////

        ////////////////////////////////////////////// Functions /////////////////////////////////////////////
        ////////////////////////////// Lidar ////////////////////////////////
        void CreateLidar();

        bool LoadEntityToRGL(
                const ignition::gazebo::Entity& entity,
                const ignition::gazebo::components::Visual*,
                const ignition::gazebo::components::Geometry* geometry);

        bool RemoveEntityFromRGL(
                const ignition::gazebo::Entity& entity,
                const ignition::gazebo::components::Visual*,
                const ignition::gazebo::components::Geometry*);

        void RayTrace(ignition::gazebo::EntityComponentManager& ecm);

        void UpdateRGLEntityPose(const ignition::gazebo::EntityComponentManager& ecm);

        void UpdateLidarPose(const ignition::gazebo::EntityComponentManager& ecm);

        ////////////////////////////// Mesh /////////////////////////////////

        const ignition::common::Mesh* LoadBox(
                const ignition::gazebo::components::Geometry* geometry,
                double& scale_x,
                double& scale_y,
                double& scale_z);

        const ignition::common::Mesh* LoadCapsule(const ignition::gazebo::components::Geometry* geometry);

        const ignition::common::Mesh* LoadCylinder(
                const ignition::gazebo::components::Geometry* geometry,
                double& scale_x,
                double& scale_y,
                double& scale_z);

        const ignition::common::Mesh* LoadEllipsoid(
                const ignition::gazebo::components::Geometry* geometry,
                double& scale_x,
                double& scale_y,
                double& scale_z);

        // the mathematical kind of plane
        const ignition::common::Mesh* LoadPlane(
                const ignition::gazebo::components::Geometry* geometry,
                double& scale_x,
                double& scale_y);

        const ignition::common::Mesh* LoadSphere(
                const ignition::gazebo::components::Geometry* geometry,
                double& scale_x,
                double& scale_y,
                double& scale_z);

        // also gets the scale of the mesh
        const ignition::common::Mesh* GetMeshPointer(
                const ignition::gazebo::components::Geometry* geometry,
                double& scale_x,
                double& scale_y,
                double& scale_z);

        bool GetMesh(
                const ignition::gazebo::components::Geometry* geometry,
                int& vertex_count,
                int& triangle_count,
                rgl_vec3f*& vertices,
                rgl_vec3i** triangles);

        bool LoadMeshToRGL(rgl_mesh_t* new_mesh, const ignition::gazebo::components::Geometry* geometry);

        ////////////////////////////// Utils /////////////////////////////////

        static ignition::math::Pose3<double> FindWorldPose(
                const ignition::gazebo::Entity& entity,
                const ignition::gazebo::EntityComponentManager& ecm);

        // get the local to global transform matrix
        static rgl_mat3x4f GetRglMatrix(ignition::gazebo::Entity entity,
                                        const ignition::gazebo::EntityComponentManager& ecm);

        // gazebo mesh factory had problems with adding multiple floats becoming unreliable
        static float RoundFloat(float value);
    };
}