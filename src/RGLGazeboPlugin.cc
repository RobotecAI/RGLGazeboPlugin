#include <RGLGazeboPlugin.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Mesh.hh>
#include <rgl/api/experimental.h>
#include <rgl/api/e2e_extensions.h>

#define RGL_CHECK(call)                  \
do {                                     \
	rgl_status_t status = call;          \
	if (status != RGL_SUCCESS) {         \
		const char* msg;                 \
		rgl_get_last_error_string(&msg); \
		ignmsg << msg;                   \
	}                                    \
} while(0)

#define WORLD_ENTITY_ID 1

IGNITION_ADD_PLUGIN(
	rgl::RGLGazeboPlugin,
	ignition::gazebo::System,
    rgl::RGLGazeboPlugin::ISystemPreUpdate,
	rgl::RGLGazeboPlugin::ISystemPostUpdate
)

using namespace rgl;

RGLGazeboPlugin::RGLGazeboPlugin() = default;

RGLGazeboPlugin::~RGLGazeboPlugin() = default;

void RGLGazeboPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                      ignition::gazebo::EntityComponentManager &_ecm) {
    if (_info.paused) return;
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

    /// DEBUG (pasted from apiExample)
//    rgl_mesh_t cube_mesh = 0;
//    RGL_CHECK(rgl_mesh_create(&cube_mesh, cube_vertices, cube_vertices_length, cube_indices, cube_indices_length));
//
//    // Put an entity on the default scene
//    rgl_entity_t cube_entity = 0;
//    RGL_CHECK(rgl_entity_create(&cube_entity, NULL, cube_mesh));
//
//    // Set position of the cube entity to (0, 0, 5)
//    rgl_mat3x4f entity_tf = {
//            .value = {
//                    {1, 0, 0, 0},
//                    {0, 1, 0, 0},
//                    {0, 0, 1, 5}
//            }
//    };
//    RGL_CHECK(rgl_entity_set_pose(cube_entity, &entity_tf));
//
//    // Create a description of lidar that sends 1 ray
//    // By default, lidar will have infinite ray range
//    // and will be placed in (0, 0, 0), facing positive Z
//    rgl_lidar_t lidar = 0;
//    rgl_mat3x4f ray_tf = {
//            .value = {
//                    {1, 0, 0, 0},
//                    {0, 1, 0, 0},
//                    {0, 0, 1, 0},
//            }
//    };
//    RGL_CHECK(rgl_lidar_create(&lidar, &ray_tf, 1));
//
//    // Start raytracing on the default scene
//    RGL_CHECK(rgl_lidar_raytrace_async(NULL, lidar));
//
//    // Wait for raytracing (if needed) and collect results
//    int hitpoint_count = 0;
//    rgl_vec3f results[1] = {0};
//    RGL_CHECK(rgl_lidar_get_output_size(lidar, &hitpoint_count));
//    RGL_CHECK(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, results));
//
//    printf("Got %d hitpoint(s)\n", hitpoint_count);
//    for (int i = 0; i < hitpoint_count; ++i) {
//        printf("- (%.2f, %.2f, %.2f)\n", results[i].value[0], results[i].value[1], results[i].value[2]);
//    }
    /// DEBUG END
}

void RGLGazeboPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                 const ignition::gazebo::EntityComponentManager &_ecm) {
    if (_info.paused) return;

    auto LoadEntitiesToRGL = [&](const ignition::gazebo::Entity &_entity,
                                 const ignition::gazebo::components::Visual*,
                                 const ignition::gazebo::components::Geometry* _geometry) -> bool {
        int v_count;
        int i_count;
        rgl_vec3f* vertices;
        rgl_vec3i* indices;
        if (!GetMesh(_geometry, v_count, i_count, vertices, indices)) return true;
        rgl_mesh_t new_mesh;
        RGL_CHECK(rgl_mesh_create(&new_mesh, vertices, v_count, indices, i_count));
        free(vertices);
        free(indices);
        rgl_entity_t new_rgl_entity;
        RGL_CHECK(rgl_entity_create(&new_rgl_entity, nullptr, new_mesh));
        entities_in_rgl.insert(std::make_pair(_entity, std::make_pair(new_rgl_entity, new_mesh)));
        ignmsg << "Added entity: " << _entity << std::endl;
        return true;
    };

    auto RemoveEntityInRGL = [&](const ignition::gazebo::Entity &_entity,
                               const ignition::gazebo::components::Visual*,
                               const ignition::gazebo::components::Geometry* _geometry) -> bool {
        if (!EntityInRGL(_entity)) return true;
        RGL_CHECK(rgl_entity_destroy(entities_in_rgl.at(_entity).first));
        RGL_CHECK(rgl_mesh_destroy(entities_in_rgl.at(_entity).second));
        entities_in_rgl.erase(_entity);
        ignmsg << "Entity removed: "<< _entity << std::endl;
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

    _ecm.EachRemoved<ignition::gazebo::components::Visual,
            ignition::gazebo::components::Geometry> (RemoveEntityInRGL);

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
                              int& v_count, int& i_count, rgl_vec3f*& vertices, rgl_vec3i*& indices) {
    auto mesh_sdf = geometry->Data().MeshShape();
    if (mesh_sdf == nullptr) return false;

    auto mesh_common = mesh_manager->MeshByName(mesh_sdf->Uri());
    if (mesh_common == nullptr) return false;

    unsigned int u_ver_count = mesh_common->VertexCount();
    assert(u_ver_count <= INT_MAX);
    unsigned int u_ind_count = mesh_common->IndexCount();
    assert(u_ind_count <= INT_MAX);

    v_count = static_cast<int>(u_ver_count / 3);
    i_count = static_cast<int>(u_ind_count / 3);

    vertices = static_cast<rgl_vec3f*>(malloc(sizeof(rgl_vec3f) * v_count));
    indices = static_cast<rgl_vec3i*>(malloc(sizeof(rgl_vec3i) * i_count));
    auto vertices_double_arr = static_cast<double*>(malloc(sizeof(double) * (v_count * 3)));

    mesh_common->FillArrays(&vertices_double_arr, (int**)(&indices));

    int v_index = 0;

    for (int i = 0; i < v_count; ++i) {
        for (int j = 0; j < 3; ++j) {
            vertices[i].value[j] = static_cast<float>(vertices_double_arr[v_index]);
            v_index++;
        }
    }

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
//    ignmsg << "entity: " << gazebo_lidar << " rgl_pose_matrix: " << std::endl;
//    for (int i = 0; i < 3; ++i) {
//        for (int j = 0; j < 4; ++j) {
//            ignmsg << rgl_pose_matrix.value[i][j] << " ";
//        }
//        ignmsg << std::endl;
//    }
}