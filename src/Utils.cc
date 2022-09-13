#include <RGLGazeboPlugin.hh>

#define ROUND_BY_VALUE 10000

using namespace rgl;

ignition::math::Pose3<double> RGLGazeboPlugin::FindWorldPose(
        const ignition::gazebo::Entity& entity,
        const ignition::gazebo::EntityComponentManager& ecm) {

    auto local_pose = ecm.Component<ignition::gazebo::components::Pose>(entity);
    assert(local_pose != nullptr);
    auto world_pose = local_pose->Data();

    ignition::gazebo::Entity this_entity = entity;
    ignition::gazebo::Entity parent;

    while ((parent = ecm.ParentEntity(this_entity)) != WORLD_ENTITY_ID) {
        auto parent_pose = ecm.Component<ignition::gazebo::components::Pose>(parent);
        assert(parent_pose != nullptr);
        world_pose += parent_pose->Data();
        this_entity = parent;
    }

    return world_pose;
}

rgl_mat3x4f RGLGazeboPlugin::GetRglMatrix(
        ignition::gazebo::Entity entity,
        const ignition::gazebo::EntityComponentManager& ecm) {

    auto gazebo_matrix = ignition::math::Matrix4<double>(FindWorldPose(entity, ecm));

    /// Debug printf
//    ignmsg << "entity: " << entity << " gazebo_matrix: " << gazebo_matrix << std::endl;

    rgl_mat3x4f rgl_matrix;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            rgl_matrix.value[i][j] = static_cast<float>(gazebo_matrix(i, j));
        }
    }

    return rgl_matrix;
}


float RGLGazeboPlugin::RoundFloat(float value) {
    return std::roundf(value * ROUND_BY_VALUE) / ROUND_BY_VALUE;
}