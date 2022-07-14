#include <RGLGazeboPlugin.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Mesh.hh>

#define WORLD_ENTITY_ID 1

IGNITION_ADD_PLUGIN(
	rgl::RGLGazeboPlugin,
	ignition::gazebo::System,
    rgl::RGLGazeboPlugin::ISystemPreUpdate,
	rgl::RGLGazeboPlugin::ISystemPostUpdate
)

using namespace std::literals::chrono_literals;
using namespace rgl;

RGLGazeboPlugin::RGLGazeboPlugin() = default;

RGLGazeboPlugin::~RGLGazeboPlugin() = default;

void RGLGazeboPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                      ignition::gazebo::EntityComponentManager &_ecm) {
    this->simTime = _info.simTime;

    if (simTime == 0s) {
        return;
    }

    if (rgl_entity == 0) {
        rgl_entity = _ecm.CreateEntity();
    }

    // TODO: import result mesh from RGL
}

void RGLGazeboPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                 const ignition::gazebo::EntityComponentManager &_ecm) {
    if (simTime == 0s) {
        return;
    }

    auto LoadEntitiesToRGL = [&](const ignition::gazebo::Entity &_entity,
                                 const ignition::gazebo::components::Visual *,
                                 const ignition::gazebo::components::Geometry *_geometry) -> bool {
        auto mesh_sdf = _geometry->Data().MeshShape();
        if (mesh_sdf == nullptr) return true;
        auto mesh_common = meshManager->MeshByName(mesh_sdf->Uri());
        if (mesh_common == nullptr) return true;
        if (mesh_names.find(mesh_common->Name()) == mesh_names.end()) {
            // TODO: add mesh to RGL here
        }
        mesh_names.insert(mesh_common->Name());
        auto world_pose = FindWorldPose(_entity, _ecm);
        // TODO: export entity to RGL
        ignmsg << "mesh: " << mesh_common->Name() << std::endl;
        ignmsg << "scale: " << mesh_sdf->Scale() << std::endl;
        ignmsg << "pose: " << world_pose.Pos() << std::endl;
        return true;
    };

    static bool entities_loaded_at_start = false;
    if (!entities_loaded_at_start) {
        _ecm.Each<ignition::gazebo::components::Visual,
                ignition::gazebo::components::Geometry> (LoadEntitiesToRGL);
        entities_loaded_at_start = true;
    }

    _ecm.EachNew<ignition::gazebo::components::Visual,
              ignition::gazebo::components::Geometry> (LoadEntitiesToRGL);
}

ignition::math::Pose3<double> RGLGazeboPlugin::FindWorldPose(const ignition::gazebo::Entity &_entity,
                                                             const ignition::gazebo::EntityComponentManager &_ecm) {
    auto local_pose = _ecm.Component<ignition::gazebo::components::Pose>(_entity);
    assert(local_pose != nullptr);
    auto world_pose = local_pose->Data();
    ignition::gazebo::Entity this_entity = _entity;
    ignition::gazebo::Entity parent;
    while ((parent = _ecm.ParentEntity(this_entity)) != WORLD_ENTITY_ID) {
        auto parent_pose = _ecm.Component<ignition::gazebo::components::Pose>(parent);
        assert(parent_pose != nullptr);
        world_pose += parent_pose->Data();
        this_entity = parent;
    }
    return world_pose;
}
