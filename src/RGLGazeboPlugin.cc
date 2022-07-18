#include <RGLGazeboPlugin.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Mesh.hh>
#include <rgl/api/experimental.h>
#include <rgl/api/e2e_extensions.h>

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
    sim_time = _info.simTime;

    if (sim_time == 0s) {
        return;
    }

    if (lidar == 0) {
        lidar = _ecm.CreateEntity();
    }

    // TODO: import and render result mesh from RGL
    int major;
    int minor;
    int patch;
    rgl_get_version_info(&major, &minor, &patch);
    ignmsg << "RGL version: " << major << "." << minor << "." << patch << std::endl;
}

void RGLGazeboPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                 const ignition::gazebo::EntityComponentManager &_ecm) {
    if (sim_time == 0s) {
        return;
    }

    auto LoadEntitiesToRGL = [&](const ignition::gazebo::Entity &_entity,
                                 const ignition::gazebo::components::Visual*,
                                 const ignition::gazebo::components::Geometry* _geometry) -> bool {
        unsigned int v_count;
        unsigned int i_count;
        float* vertices;
        int* indices;
        if (!GetMesh(_geometry, &v_count, &i_count, &vertices, &indices)) return true;
        entities_in_rgl.insert(_entity);
        // TODO: add mesh to RGL here
        ignmsg << "mesh added entity id: " << _entity << std::endl;
        free(vertices);
        free(indices);
        // TODO: export entity to RGL
        ignmsg << "Added entity: " << _entity << std::endl;
        return true;
    };

    auto RemoveEntityInRGL = [&](const ignition::gazebo::Entity &_entity,
                               const ignition::gazebo::components::Visual*,
                               const ignition::gazebo::components::Geometry* _geometry) -> bool {
        if (!EntityInRGL(_entity)) return true;
        entities_in_rgl.erase(_entity);
        // TODO: remove entity and its mesh in RGL
        ignmsg << "Entity removed id: "<< _entity << std::endl;
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

    for (auto entity : entities_in_rgl) {
        auto pose_matrix = ignition::math::Matrix4<double>(FindWorldPose(entity, _ecm));
        // TODO: update Pose in RGL
        ignmsg << "pose_matrix: " << pose_matrix << std::endl;
    }

    _ecm.EachRemoved<ignition::gazebo::components::Visual,
            ignition::gazebo::components::Geometry> (RemoveEntityInRGL);
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

bool RGLGazeboPlugin::EntityInRGL(ignition::gazebo::Entity entity) {
    return entities_in_rgl.find(entity) != entities_in_rgl.end();
}

bool RGLGazeboPlugin::GetMesh(const ignition::gazebo::components::Geometry* geometry,
                              unsigned int* v_count, unsigned int* i_count, float** vertices, int** indices) {
    auto mesh_sdf = geometry->Data().MeshShape();
    if (mesh_sdf == nullptr) return false;

    auto mesh_common = mesh_manager->MeshByName(mesh_sdf->Uri());
    if (mesh_common == nullptr) return false;

    (*v_count) = mesh_common->VertexCount();
    (*i_count) = mesh_common->IndexCount();
    (*vertices) = static_cast<float*>(malloc(sizeof(float) * (*v_count)));
    (*indices) = static_cast<int*>(malloc(sizeof(int) * (*i_count)));
    auto vertices_double = static_cast<double*>(malloc(sizeof(double) * (*v_count)));

    mesh_common->FillArrays(&vertices_double, indices);

    for (size_t i = 0; i < *v_count; ++i) {
        (*vertices)[i] = static_cast<float>(vertices_double[i]);
    }

    free(vertices_double);
    return true;
}
