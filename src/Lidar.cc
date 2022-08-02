#include <RGLGazeboPlugin.hh>

using namespace rgl;

void RGLGazeboPlugin::CreateLidar() {
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

// always returns true, because the ecm will stop if it encounters false
bool RGLGazeboPlugin::LoadEntityToRGL(
        const ignition::gazebo::Entity& entity,
        const ignition::gazebo::components::Visual*,
        const ignition::gazebo::components::Geometry* geometry) {

    if (lidar_ignore.contains(entity)) return true;
    if (entities_in_rgl.contains(entity)) return true;
    rgl_mesh_t new_mesh;
    if (!LoadMeshToRGL(&new_mesh, geometry)) return true;
    rgl_entity_t new_rgl_entity;
    RGL_CHECK(rgl_entity_create(&new_rgl_entity, nullptr, new_mesh));
    entities_in_rgl.insert(std::make_pair(entity, std::make_pair(new_rgl_entity, new_mesh)));
    ignmsg << "Added entity: " << entity << std::endl;
    return true;
}

// always returns true, because the ecm will stop if it encounters false
bool RGLGazeboPlugin::RemoveEntityFromRGL(
        const ignition::gazebo::Entity& entity,
        const ignition::gazebo::components::Visual*,
        const ignition::gazebo::components::Geometry*) {

    // detects if gazebo lidar model is removed
    if (lidar_ignore.contains(entity)) {
        gazebo_lidar_exists = false;
        return true;
    }
    if (!entities_in_rgl.contains(entity)) return true;
    RGL_CHECK(rgl_entity_destroy(entities_in_rgl.at(entity).first));
    RGL_CHECK(rgl_mesh_destroy(entities_in_rgl.at(entity).second));
    entities_in_rgl.erase(entity);
    ignmsg << "Removed entity: " << entity << std::endl;
    return true;
}

void RGLGazeboPlugin::UpdateRGLEntityPose(const ignition::gazebo::EntityComponentManager& ecm) {

    for (auto entity: entities_in_rgl) {
        auto rgl_matrix = GetRglMatrix(entity.first, ecm);
        RGL_CHECK(rgl_entity_set_pose(entity.second.first, &rgl_matrix));

        /// Debug printf
//        ignmsg << "entity: " << entity.first << " rgl_matrix: " << std::endl;
//        for (int i = 0; i < 3; ++i) {
//            for (int j = 0; j < 4; ++j) {
//                ignmsg << rgl_matrix.value[i][j] << " ";
//            }
//            ignmsg << std::endl;
//        }

    }
}

void RGLGazeboPlugin::UpdateLidarPose(const ignition::gazebo::EntityComponentManager& ecm) {
    auto rgl_pose_matrix = GetRglMatrix(gazebo_lidar, ecm);
    RGL_CHECK(rgl_lidar_set_pose(rgl_lidar, &rgl_pose_matrix));

    /// Debug printf
//    ignmsg << "lidar: " << gazebo_lidar << " rgl_pose_matrix: " << std::endl;
//    for (int i = 0; i < 3; ++i) {
//        for (int j = 0; j < 4; ++j) {
//            ignmsg << rgl_pose_matrix.value[i][j] << " ";
//        }
//        ignmsg << std::endl;
//    }

}

void RGLGazeboPlugin::RayTrace(ignition::gazebo::EntityComponentManager& ecm) {

    RGL_CHECK(rgl_lidar_raytrace_async(nullptr, rgl_lidar));

    int hitpoint_count = 0;
    RGL_CHECK(rgl_lidar_get_output_size(rgl_lidar, &hitpoint_count));

    auto results = static_cast<rgl_vec3f*>(malloc(hitpoint_count * sizeof(rgl_vec3f)));
    RGL_CHECK(rgl_lidar_get_output_data(rgl_lidar, RGL_FORMAT_XYZ, results));

    ignmsg << "Lidar id: " << lidar_id << " Got " << hitpoint_count << " hitpoint(s)\n";
    for (int i = 0; i < hitpoint_count; ++i) {
        ignmsg << " hit: " << i << " " << results[i].value[0] << "," << results[i].value[1] << ","
               << results[i].value[2] << std::endl;
    }

    free(results);

    // TODO: render result mesh from RGL
//    mesh_manager->RemoveMesh("rgl_visual");
//    ignition::common::SubMesh new_submesh;
//    for (int i = 0; i < hitpoint_count; ++i) {
//        new_submesh.AddVertex(results[i].value[0], results[i].value[1], results[i].value[2]);
//    }
//    new_submesh.SetPrimitiveType(ignition::common::SubMesh::POINTS);
//    auto new_mesh = new ignition::common::Mesh();
//    new_mesh->AddSubMesh(new_submesh);
//    new_mesh->SetName("rgl_visual");
//    mesh_manager->AddMesh(new_mesh);
//    ecm.SetChanged(rgl_visual, ignition::gazebo::components::Geometry().TypeId());
}