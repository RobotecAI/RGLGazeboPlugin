// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/SystemPluginInfo.hh>

#include "RGLServerPluginManager.hh"

#define RGL_INSTANCE "rgl::RGLServerPluginInstance"

namespace rgl
{

#pragma clang diagnostic push
#pragma ide diagnostic ignored "ConstantFunctionResult"

// always returns true, because the ecm will stop if it encounters false
bool RGLServerPluginManager::RegisterNewLidarCb(
        gz::sim::Entity entity,
        const gz::sim::EntityComponentManager& ecm)
{
    // Plugin must be inside CustomSensor
    if (!ecm.EntityHasComponentType(entity, gz::sim::components::CustomSensor::typeId))
    {
        return true;
    }

    // Looking for plugin
    auto pluginData = ecm.ComponentData<gz::sim::components::SystemPluginInfo>(entity);
    if (pluginData == std::nullopt) {
        return true;
    }
    auto plugins = pluginData->plugins();
    for (const auto& plugin : plugins) {
        if (plugin.name() == RGL_INSTANCE) {
            lidarEntities.insert(entity);
            for (auto descendant: ecm.Descendants(entity)) {
                entitiesToIgnore.insert(descendant);
            }
        }
    }

    // RGL lidar plugin not found
    if (!lidarEntities.contains(entity)) {
        return true;
    }

    // Ignore all entities in link associated with RGL lidar
    // Link could contain visual representation of the lidar
    for (auto entityInParentLink : GetEntitiesInParentLink(entity, ecm)) {
        entitiesToIgnore.insert(entityInParentLink);
    }

    return true;
}

// always returns true, because the ecm will stop if it encounters false
bool RGLServerPluginManager::UnregisterLidarCb(
        gz::sim::Entity entity,
        const gz::sim::EntityComponentManager& ecm)
{
    if (!lidarEntities.contains(entity)) {
        return true;
    }
    for (auto entityInParentLink : GetEntitiesInParentLink(entity, ecm)) {
        entitiesToIgnore.erase(entityInParentLink);
    }
    lidarEntities.erase(entity);
    return true;
}

// always returns true, because the ecm will stop if it encounters false
bool RGLServerPluginManager::LoadEntityToRGLCb(
        const gz::sim::Entity& entity,
        const gz::sim::components::Visual*,
        const gz::sim::components::Geometry* geometry)
{
    if (entitiesToIgnore.contains(entity)) {
        return true;
    }
    if (entitiesInRgl.contains(entity)) {
        ignwarn << "Trying to add same entity (" << entity << ") to rgl multiple times!\n";
        return true;
    }
    rgl_mesh_t rglMesh;
    if (!LoadMeshToRGL(&rglMesh, geometry->Data())) {
        ignerr << "Failed to load mesh to RGL from entity (" << entity << "). Skipping...\n";
        return true;
    }
    rgl_entity_t rglEntity;
    if (!CheckRGL(rgl_entity_create(&rglEntity, nullptr, rglMesh))) {
        ignerr << "Failed to load entity (" << entity << ") to RGL. Skipping...\n";
        return true;
    }
    entitiesInRgl.insert({entity, {rglEntity, rglMesh}});
    return true;
}

// always returns true, because the ecm will stop if it encounters false
bool RGLServerPluginManager::RemoveEntityFromRGLCb(
        const gz::sim::Entity& entity,
        const gz::sim::components::Visual*,
        const gz::sim::components::Geometry*)
{
    if (entitiesToIgnore.contains(entity)) {
        entitiesToIgnore.erase(entity);
        return true;
    }
    if (!entitiesInRgl.contains(entity)) {
        return true;
    }
    if (!CheckRGL(rgl_entity_destroy(entitiesInRgl.at(entity).first))) {
        ignerr << "Failed to remove entity (" << entity << ") from RGL.\n";
    }
    if (!CheckRGL(rgl_mesh_destroy(entitiesInRgl.at(entity).second))) {
        ignerr << "Failed to remove mesh from entity (" << entity << ") in RGL.\n";
    }
    entitiesInRgl.erase(entity);
    return true;
}
#pragma clang diagnostic pop

void RGLServerPluginManager::UpdateRGLEntityPoses(const gz::sim::EntityComponentManager& ecm)
{
    for (auto entity: entitiesInRgl) {
        rgl_mat3x4f rglMatrix = FindWorldPoseInRglMatrix(entity.first, ecm);
        if (!CheckRGL(rgl_entity_set_pose(entity.second.first, &rglMatrix))) {
            ignerr << "Failed to update pose for entity (" << entity.first << ").\n";
        }
    }
}

std::unordered_set<gz::sim::Entity> RGLServerPluginManager::GetEntitiesInParentLink(
        gz::sim::Entity entity,
        const gz::sim::EntityComponentManager& ecm)
{
    auto parentEntity = ecm.ParentEntity(entity);
    if (parentEntity == gz::sim::kNullEntity ||
        !ecm.EntityHasComponentType(parentEntity, gz::sim::components::Link::typeId)) {
        return {};
    }
    return ecm.Descendants(parentEntity);
}

}  // namespace rgl
