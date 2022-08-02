#include <RGLGazeboPlugin.hh>

#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/EntityComponentManager.hh>

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
        rgl::RGLGazeboPlugin,
        ignition::gazebo::System,
        rgl::RGLGazeboPlugin::ISystemConfigure,
        rgl::RGLGazeboPlugin::ISystemPreUpdate,
        rgl::RGLGazeboPlugin::ISystemPostUpdate
)

using namespace rgl;
using namespace std::literals::chrono_literals;
using namespace std::placeholders;

RGLGazeboPlugin::RGLGazeboPlugin() = default;

RGLGazeboPlugin::~RGLGazeboPlugin() = default;

void RGLGazeboPlugin::Configure(
        const ignition::gazebo::Entity& entity,
        const std::shared_ptr<const sdf::Element>&,
        ignition::gazebo::EntityComponentManager& ecm,
        ignition::gazebo::EventManager&) {

    ignmsg << "attached to: " << entity << std::endl;
    gazebo_lidar = entity;
    static int next_free_id = 0;
    lidar_id = next_free_id;
    next_free_id++;

    for (auto descendant: ecm.Descendants(gazebo_lidar)) {
        lidar_ignore.insert(descendant);
    };

    rgl_visual = ecm.CreateEntity();
    lidar_ignore.insert(rgl_visual);

    ecm.Each<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            (std::bind(&RGLGazeboPlugin::LoadEntityToRGL, this, _1, _2, _3));

    /// experiments with adding visual indication of hit points in gazebo GUI
//    ignmsg << "rgl visual: " << rgl_visual << std::endl;
//    auto common_empty_mesh = new ignition::common::Mesh();
//    common_empty_mesh->SetName("rgl_visual");
//    mesh_manager->AddMesh(common_empty_mesh);
//    sdf::Mesh sdf_empty_mesh;
//    sdf_empty_mesh.SetUri("rgl_visual");
//    sdf::Geometry sdf_geometry;
//    sdf_geometry.SetType(sdf::GeometryType::MESH);
//    sdf_geometry.SetMeshShape(sdf_empty_mesh);
//    auto component_geometry = ignition::gazebo::components::Geometry(sdf_geometry);
//
//    ecm.CreateComponent(rgl_visual, component_geometry);
//    ecm.CreateComponent(rgl_visual, ignition::gazebo::components::Visual());
//    ecm.CreateComponent(rgl_visual, ignition::gazebo::components::Name("rgl_visual"));
//    ecm.CreateComponent(rgl_visual, ignition::gazebo::components::Pose(ignition::math::Pose3d(0, 0, 0, 0, 0, 0)));
//    ecm.SetParentEntity(rgl_visual, WORLD_ENTITY_ID);
    ///end of experiments
}

void RGLGazeboPlugin::PreUpdate(
        const ignition::gazebo::UpdateInfo& info,
        ignition::gazebo::EntityComponentManager& ecm) {

    if (!lidar_created) {
        CreateLidar();
        lidar_created = true;
        return;
    }

    if (!ray_trace) return;
    ray_trace = false;

    RayTrace(ecm);
}

void RGLGazeboPlugin::PostUpdate(
        const ignition::gazebo::UpdateInfo& info,
        const ignition::gazebo::EntityComponentManager& ecm) {

    ecm.EachNew<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            (std::bind(&RGLGazeboPlugin::LoadEntityToRGL, this, _1, _2, _3));

    ecm.EachRemoved<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            (std::bind(&RGLGazeboPlugin::RemoveEntityFromRGL, this, _1, _2, _3));

    if (info.simTime < last_update + 100ms) return;
    last_update = info.simTime;
    ray_trace = true;

    UpdateRGLEntityPose(ecm);

    UpdateLidarPose(ecm);
}