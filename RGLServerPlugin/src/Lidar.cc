#include "RGLServerPlugin.hh"

#include <ignition/gazebo/components/VisualCmd.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/msgs/visual.pb.h>
#include <ignition/msgs/geometry.pb.h>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/msgs/Utility.hh>
#include <sdf/Mesh.hh>
#include <ignition/common/SubMesh.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/msgs/pointcloud_packed.pb.h>
#include <ignition/transport/Node.hh>
#include <fstream>

#define RAYS_IN_ONE_DIR 1000

using namespace rgl;

void RGLServerPlugin::CreateLidar() {
    rgl_configure_logging(RGL_LOG_LEVEL_DEBUG, nullptr, true);
    size_t rays = RAYS_IN_ONE_DIR * RAYS_IN_ONE_DIR;
    std::vector<rgl_mat3x4f> ray_tf;
    ignition::math::Angle X;
    ignition::math::Angle Y;
    double unit_angle = ignition::math::Angle::TwoPi.Radian() / RAYS_IN_ONE_DIR;
    for (int i = 0; i < RAYS_IN_ONE_DIR; ++i, X.SetRadian(unit_angle * i)) {
        for (int j = 0; j < RAYS_IN_ONE_DIR; ++j, Y.SetRadian(unit_angle * j)) {
            ignition::math::Quaterniond quaternion(0, X.Radian(), Y.Radian());
            ignition::math::Matrix4d matrix4D(quaternion);
            rgl_mat3x4f rgl_matrix;
            for (int l = 0; l < 3; ++l) {
                for (int m = 0; m < 4; ++m) {
                    rgl_matrix.value[l][m] = static_cast<float>(matrix4D(l, m));
                }
            }
            ray_tf.push_back(rgl_matrix);
        }
    }
    RGL_CHECK(rgl_lidar_create(&rgl_lidar, ray_tf.data(), rays));
    pcPub = node.Advertise<ignition::msgs::PointCloudPacked>("/point_cloud");
}

// always returns true, because the ecm will stop if it encounters false
bool RGLServerPlugin::LoadEntityToRGL(
        const ignition::gazebo::Entity& entity,
        const ignition::gazebo::components::Visual*,
        const ignition::gazebo::components::Geometry* geometry) {

    if (lidar_ignore.contains(entity)) return true;
    if (entities_in_rgl.contains(entity)) return true;
    rgl_mesh_t new_mesh;
    if (!LoadMeshToRGL(&new_mesh, geometry->Data())) return true;
    rgl_entity_t new_rgl_entity;
    RGL_CHECK(rgl_entity_create(&new_rgl_entity, nullptr, new_mesh));
    entities_in_rgl.insert(std::make_pair(entity, std::make_pair(new_rgl_entity, new_mesh)));
    ignmsg << "Added entity: " << entity << std::endl;
    return true;
}

// always returns true, because the ecm will stop if it encounters false
bool RGLServerPlugin::RemoveEntityFromRGL(
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

void RGLServerPlugin::UpdateRGLEntityPose(const ignition::gazebo::EntityComponentManager& ecm) {
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

void RGLServerPlugin::UpdateLidarPose(const ignition::gazebo::EntityComponentManager& ecm) {
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

void RGLServerPlugin::RayTrace(ignition::gazebo::EntityComponentManager& ecm) {
    auto start_raytrace = std::chrono::system_clock::now();

    RGL_CHECK(rgl_lidar_raytrace_async(nullptr, rgl_lidar));

    auto end_raytrace = std::chrono::system_clock::now();
    auto time_to_raytrace = std::chrono::duration_cast<std::chrono::milliseconds>(end_raytrace - start_raytrace);
    ignmsg << "raytrace time: " << time_to_raytrace.count() << " ms\n";

    int hitpoint_count = 0;
    RGL_CHECK(rgl_lidar_get_output_size(rgl_lidar, &hitpoint_count));
    if (hitpoint_count == 0) return;
    std::vector<rgl_vec3f> results(hitpoint_count, rgl_vec3f());
    RGL_CHECK(rgl_lidar_get_output_data(rgl_lidar, RGL_FORMAT_XYZ, results.data()));

    auto end_get_results = std::chrono::system_clock::now();
    auto time_to_get_results = std::chrono::duration_cast<std::chrono::milliseconds>(end_get_results - end_raytrace);
    ignmsg << "get results time: " << time_to_get_results.count() << " ms\n";

    ignmsg << "Lidar id: " << lidar_id << " Got " << hitpoint_count << " hitpoint(s)\n";
//    for (int i = 0; i < hitpoint_count; ++i) {
//        ignmsg << " hit: " << i << " coordinates: " << results[i].value[0] << "," << results[i].value[1] << ","
//               << results[i].value[2] << std::endl;
//    }

    // Create messages
    ignition::msgs::PointCloudPacked point_cloud_msg;
    ignition::msgs::InitPointCloudPacked(point_cloud_msg, "some_frame", true,
                                         {{"xyz", ignition::msgs::PointCloudPacked::Field::FLOAT32}});
    point_cloud_msg.mutable_data()->resize(hitpoint_count * point_cloud_msg.point_step());
    point_cloud_msg.set_height(1);
    point_cloud_msg.set_width(hitpoint_count);

    // Populate messages
    ignition::msgs::PointCloudPackedIterator<float> xIter(point_cloud_msg, "x");
    ignition::msgs::PointCloudPackedIterator<float> yIter(point_cloud_msg, "y");
    ignition::msgs::PointCloudPackedIterator<float> zIter(point_cloud_msg, "z");

    for (int i = 0; i < hitpoint_count; ++i, ++xIter, ++yIter, ++zIter)
    {
        *xIter = results[i].value[0];
        *yIter = results[i].value[1];
        *zIter = results[i].value[2];
    }

    auto end_populate_msg = std::chrono::system_clock::now();
    auto time_to_populate_msg = std::chrono::duration_cast<std::chrono::milliseconds>(end_populate_msg - end_get_results);
    ignmsg << "populate message time: " << time_to_populate_msg.count() << " ms\n";

    pcPub.Publish(point_cloud_msg);

    auto end_sending_msg = std::chrono::system_clock::now();
    auto time_to_send_msg = std::chrono::duration_cast<std::chrono::milliseconds>(end_sending_msg - end_populate_msg);
    ignmsg << "send message time: " << time_to_send_msg.count() << " ms\n";
}