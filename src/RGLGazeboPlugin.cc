#include <RGLGazeboPlugin.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Mesh.hh>
#include <rgl/api/experimental.h>
#include <ignition/common/SubMesh.hh>

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

#define WORLD_ENTITY_ID 1

IGNITION_ADD_PLUGIN(
	rgl::RGLGazeboPlugin,
	ignition::gazebo::System,
    rgl::RGLGazeboPlugin::ISystemConfigure,
    rgl::RGLGazeboPlugin::ISystemPreUpdate,
	rgl::RGLGazeboPlugin::ISystemPostUpdate
)

using namespace rgl;
using namespace std::literals::chrono_literals;

RGLGazeboPlugin::RGLGazeboPlugin() = default;

RGLGazeboPlugin::~RGLGazeboPlugin() = default;

void RGLGazeboPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                      ignition::gazebo::EntityComponentManager &_ecm) {
    if (_info.simTime == 0s) return;

    static bool lidar_created = false;
    if (!lidar_created) {
        CreateLidar(_ecm);
        lidar_created = true;
        return;
    }
    RGL_CHECK(rgl_lidar_raytrace_async(nullptr, rgl_lidar));
    int hitpoint_count = 0;
    rgl_vec3f results[1];
    RGL_CHECK(rgl_lidar_get_output_size(rgl_lidar, &hitpoint_count));
    RGL_CHECK(rgl_lidar_get_output_data(rgl_lidar, RGL_FORMAT_XYZ, results));

    ignmsg << "Got " << hitpoint_count << " hitpoint(s)\n";
    for (int i = 0; i < hitpoint_count; ++i) {
        ignmsg <<"hit: " << i << " " << results[i].value[0] << " " << results[i].value[1] << " " << results[i].value[2] << std::endl;
    }
    // TODO: render result mesh from RGL
}

void RGLGazeboPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                 const ignition::gazebo::EntityComponentManager &_ecm) {
    if (_info.simTime == 0s) return;

    auto LoadEntitiesToRGL = [&](const ignition::gazebo::Entity &_entity,
                                 const ignition::gazebo::components::Visual*,
                                 const ignition::gazebo::components::Geometry* _geometry) -> bool {
        rgl_mesh_t new_mesh;
        if (!LoadMeshToRGL(&new_mesh, _geometry)) return true;
        rgl_entity_t new_rgl_entity;
        RGL_CHECK(rgl_entity_create(&new_rgl_entity, nullptr, new_mesh));
        entities_in_rgl.insert(std::make_pair(_entity, std::make_pair(new_rgl_entity, new_mesh)));
        ignmsg << "Added entity: " << _entity << std::endl;
        return true;
    };

    auto RemoveEntityFromRGL = [&](const ignition::gazebo::Entity &_entity,
                                   const ignition::gazebo::components::Visual*,
                                   const ignition::gazebo::components::Geometry* _geometry) -> bool {
        if (!EntityInRGL(_entity)) return true;
        RGL_CHECK(rgl_entity_destroy(entities_in_rgl.at(_entity).first));
        RGL_CHECK(rgl_mesh_destroy(entities_in_rgl.at(_entity).second));
        entities_in_rgl.erase(_entity);
        ignmsg << "Entity removed: "<< _entity << std::endl;
        return true;
    };

    static bool init = false;
    if (!init) {
        _ecm.Each<ignition::gazebo::components::Visual,
                ignition::gazebo::components::Geometry> (LoadEntitiesToRGL);
        init = true;
    }

    _ecm.EachNew<ignition::gazebo::components::Visual,
            ignition::gazebo::components::Geometry> (LoadEntitiesToRGL);

    _ecm.EachRemoved<ignition::gazebo::components::Visual,
            ignition::gazebo::components::Geometry> (RemoveEntityFromRGL);

    UpdateRGLEntitiesPose(_ecm);

    UpdateLidarPose(_ecm);
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
                              int& v_count, int& i_count, rgl_vec3f*& vertices, rgl_vec3i** indices) {
    auto mesh_sdf = geometry->Data().MeshShape();
    if (mesh_sdf == nullptr) return false;

    auto mesh_common = mesh_manager->MeshByName(mesh_sdf->Uri());
    if (mesh_common == nullptr) return false;

    v_count = static_cast<int>(mesh_common->VertexCount());
    i_count = static_cast<int>(mesh_common->IndexCount() / 3);

    vertices = static_cast<rgl_vec3f*>(malloc(sizeof(rgl_vec3f) * v_count));
    double* vertices_double_arr = nullptr;

    mesh_common->FillArrays(&vertices_double_arr, (int**)indices);

    int v_index = 0;

    for (int i = 0; i < v_count; ++i) {
        for (int j = 0; j < 3; ++j) {
            vertices[i].value[j] = static_cast<float>(vertices_double_arr[v_index]);
            v_index++;
        }
    }
//    int count = 0;
//    std::cout << "vertices: ";
//    for (int i = 0; i < v_count; ++i) {
//        std::cout << count << ": ";
//        count++;
//        for (int j = 0; j < 3; ++j) {
//            std::cout << vertices[i].value[j] << ",";
//        }
//        std::cout << " ";
//    }
//    std::cout << "\n";
//
//    std::cout << "indices: ";
//    for (int i = 0; i < i_count; ++i) {
//        for (int j = 0; j < 3; ++j) {
//            std::cout << (*indices)[i].value[j] << ",";
//        }
//        std::cout << " ";
//    }
//    std::cout << "\n";
    free(vertices_double_arr);
    return true;
}

rgl_mat3x4f RGLGazeboPlugin::GetRglMatrix(ignition::gazebo::Entity entity, const ignition::gazebo::EntityComponentManager &_ecm) {
    auto gazebo_matrix = ignition::math::Matrix4<double>(FindWorldPose(entity, _ecm));
//    ignmsg << "entity: " << entity << " gazebo_matrix: " << gazebo_matrix << std::endl;
    rgl_mat3x4f rgl_matrix;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            rgl_matrix.value[i][j] = static_cast<float>(gazebo_matrix(i, j));
        }
    }
    return rgl_matrix;
}

void RGLGazeboPlugin::UpdateRGLEntitiesPose(const ignition::gazebo::EntityComponentManager &_ecm) {
    for (auto entity : entities_in_rgl) {
        auto rgl_matrix = GetRglMatrix(entity.first, _ecm);
        RGL_CHECK(rgl_entity_set_pose(entity.second.first, &rgl_matrix));
//        ignmsg << "entity: " << entity.first << " rgl_matrix: " << std::endl;
//        for (int i = 0; i < 3; ++i) {
//            for (int j = 0; j < 4; ++j) {
//                ignmsg << rgl_matrix.value[i][j] << " ";
//            }
//            ignmsg << std::endl;
//        }
    }
}

void RGLGazeboPlugin::CreateLidar(ignition::gazebo::EntityComponentManager &_ecm) {
    rgl_configure_logging(RGL_LOG_LEVEL_DEBUG, nullptr, true);
    rgl_mat3x4f ray_tf = {
            .value = {
                    {1, 0, 0, 0},
                    {0, 1, 0, 0},
                    {0, 0, 1, 0},
            }
    };
    RGL_CHECK(rgl_lidar_create(&rgl_lidar, &ray_tf, 1));
}

void RGLGazeboPlugin::UpdateLidarPose(const ignition::gazebo::EntityComponentManager &_ecm) {
    auto rgl_pose_matrix = GetRglMatrix(gazebo_lidar, _ecm);
    RGL_CHECK(rgl_lidar_set_pose(rgl_lidar, &rgl_pose_matrix));
//    ignmsg << "lidar: " << gazebo_lidar << " rgl_pose_matrix: " << std::endl;
//    for (int i = 0; i < 3; ++i) {
//        for (int j = 0; j < 4; ++j) {
//            ignmsg << rgl_pose_matrix.value[i][j] << " ";
//        }
//        ignmsg << std::endl;
//    }
}

bool RGLGazeboPlugin::LoadMeshToRGL(rgl_mesh_t* new_mesh, const ignition::gazebo::components::Geometry* geometry) {
    int v_count;
    int i_count;
    rgl_vec3f* vertices = nullptr;
    rgl_vec3i* indices = nullptr;
    if (!GetMesh(geometry, v_count, i_count, vertices, &indices)) return false;
    RGL_CHECK(rgl_mesh_create(new_mesh, vertices, v_count, indices, i_count));
    free(vertices);
    free(indices);
    return true;
}

void RGLGazeboPlugin::Configure(
        const ignition::gazebo::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &/*_sdf*/,
        ignition::gazebo::EntityComponentManager &_ecm,
        ignition::gazebo::EventManager &_eventMgr) {

    ignmsg << "attached to: " << _entity << std::endl;
}