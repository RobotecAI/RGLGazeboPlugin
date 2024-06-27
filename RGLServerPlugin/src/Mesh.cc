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

#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>
#include <gz/sim/Util.hh>

#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

#include "RGLServerPluginManager.hh"

#define UNIT_BOX_ID "unit_box"
#define UNIT_CYLINDER_ID "unit_cylinder"
#define UNIT_PLANE_ID "unit_plane"
#define UNIT_SPHERE_ID "unit_sphere"

#define CAPSULE_RINGS 32
#define CAPSULE_SEGMENTS 32

namespace rgl
{

RGLServerPluginManager::MeshInfo RGLServerPluginManager::LoadBox(
        const sdf::Geometry& data,
        double& scaleX,
        double& scaleY,
        double& scaleZ)
{
    auto size = data.BoxShape()->Size();

    scaleX = size.X();
    scaleY = size.Y();
    scaleZ = size.Z();

    return meshManager->MeshByName(UNIT_BOX_ID);
}

RGLServerPluginManager::MeshInfo RGLServerPluginManager::LoadCylinder(
        const sdf::Geometry& data,
        double& scaleX,
        double& scaleY,
        double& scaleZ)
{
    auto shape = data.CylinderShape();

    scaleX = shape->Radius() * 2;
    scaleY = shape->Radius() * 2;
    scaleZ = shape->Length();

    return meshManager->MeshByName(UNIT_CYLINDER_ID);
}

RGLServerPluginManager::MeshInfo RGLServerPluginManager::LoadEllipsoid(
        const sdf::Geometry& data,
        double& scaleX,
        double& scaleY,
        double& scaleZ)
{
    auto shape = data.EllipsoidShape()->Radii();

    scaleX = shape.X() * 2;
    scaleY = shape.Y() * 2;
    scaleZ = shape.Z() * 2;

    return meshManager->MeshByName(UNIT_SPHERE_ID);
}

// Need to handle in different way than other primitive geometry types.
// The difference is that "unit_capsule" mesh is not created by the gazebo MeshManager,
// while other primitive types have their unit meshes created and ready to query by name.
RGLServerPluginManager::MeshInfo RGLServerPluginManager::LoadCapsule(const sdf::Geometry& data)
{
    auto shape = data.CapsuleShape();

    // Unique capsule mesh name with radius and length
    std::string capsuleMeshName = "capsule_mesh";
    capsuleMeshName += "_" + std::to_string(shape->Radius())
                    + "_" + std::to_string(shape->Length());

    // Create new mesh if needed
    if (!meshManager->HasMesh(capsuleMeshName)) {
        meshManager->CreateCapsule(capsuleMeshName,
                                   shape->Radius(),
                                   shape->Length(),
                                   CAPSULE_RINGS,
                                   CAPSULE_SEGMENTS);
    }

    return meshManager->MeshByName(capsuleMeshName);
}

RGLServerPluginManager::MeshInfo RGLServerPluginManager::LoadMesh(
        const sdf::Geometry& data,
        double& scaleX,
        double& scaleY,
        double& scaleZ)
{
    auto scale = data.MeshShape()->Scale();

    scaleX = scale.X();
    scaleY = scale.Y();
    scaleZ = scale.Z();

    std::string meshPath = gz::sim::asFullPath(
            data.MeshShape()->Uri(),
            data.MeshShape()->FilePath());

    const gz::common::Mesh* mesh = meshManager->Load(meshPath);

    if (mesh == nullptr) {
        ignerr << "Failed to import mesh to RGL: " << meshPath << ".\n";
        return std::monostate{};
    }

    std::string subMeshName = data.MeshShape()->Submesh();

    if (subMeshName.empty()) {
        return mesh;
    }

    if (auto subMesh = mesh->SubMeshByName(subMeshName).lock()) {
        // subMesh must not be null
        if (subMesh.get()) {
            gz::common::SubMesh subMeshCopy(*subMesh);
            if (data.MeshShape()->CenterSubmesh()) {
                subMeshCopy.Center();
            }
            return subMeshCopy;
        }
    }
    ignerr << "Failed to import subMesh to RGL: " << subMeshName << ".\n";
    return std::monostate{};
}

RGLServerPluginManager::MeshInfo RGLServerPluginManager::LoadPlane(
        const sdf::Geometry& data,
        double& scaleX,
        double& scaleY)
{
    auto size = data.PlaneShape()->Size();

    scaleX = size.X() * 2;
    scaleY = size.Y() * 2;

    return meshManager->MeshByName(UNIT_PLANE_ID);
}

RGLServerPluginManager::MeshInfo RGLServerPluginManager::LoadSphere(
        const sdf::Geometry& data,
        double& scaleX,
        double& scaleY,
        double& scaleZ)
{
    auto radius = data.SphereShape()->Radius();

    scaleX = radius * 2;
    scaleY = radius * 2;
    scaleZ = radius * 2;

    return meshManager->MeshByName(UNIT_SPHERE_ID);
}

RGLServerPluginManager::MeshInfo RGLServerPluginManager::GetMeshPointer(
        const sdf::Geometry& data,
        double& scaleX,
        double& scaleY,
        double& scaleZ)
{
    switch (data.Type()) {
        case sdf::GeometryType::BOX:
            return LoadBox(data, scaleX, scaleY, scaleZ);
        case sdf::GeometryType::CAPSULE:
            return LoadCapsule(data);
        case sdf::GeometryType::CYLINDER:
            return LoadCylinder(data, scaleX, scaleY, scaleZ);
        case sdf::GeometryType::ELLIPSOID:
            return LoadEllipsoid(data, scaleX, scaleY, scaleZ);
        case sdf::GeometryType::EMPTY:
            return std::monostate{};
        case sdf::GeometryType::MESH:
            return LoadMesh(data, scaleX, scaleY, scaleZ);
        case sdf::GeometryType::PLANE:
            return LoadPlane(data, scaleX, scaleY);
        case sdf::GeometryType::SPHERE:
            return LoadSphere(data, scaleX, scaleY, scaleZ);
        default:
            return std::monostate{};
    }
}

bool RGLServerPluginManager::LoadMeshToRGL(
        rgl_mesh_t* mesh,
        const sdf::Geometry& data)
{
    double scaleX = 1;
    double scaleY = 1;
    double scaleZ = 1;

    auto meshInfo = GetMeshPointer(data, scaleX, scaleY, scaleZ);
    if (std::holds_alternative<std::monostate>(meshInfo)) {
        ignerr << "Failed to load mesh of geometry type '" << static_cast<int>(data.Type())
               << "' to RGL. Skipping...\n";
        return false;
    }

    int vertexCount;
    int triangleCount;
    double* ignVertices = nullptr;
    std::vector<rgl_vec3f> rglVertices;  // separated array because ign operates on doubles, and rgl on floats
    rgl_vec3i* triangles = nullptr;

    if (std::holds_alternative<gz::common::SubMesh>(meshInfo)) {
        auto ignSubMesh = get<gz::common::SubMesh>(meshInfo);
        vertexCount = static_cast<int>(ignSubMesh.VertexCount());
        triangleCount = static_cast<int>(ignSubMesh.IndexCount() / 3);
        rglVertices.reserve(vertexCount);
        ignSubMesh.FillArrays(&ignVertices, reinterpret_cast<int**>(&triangles));
    } else {
        auto ignMesh = get<const gz::common::Mesh*>(meshInfo);
        vertexCount = static_cast<int>(ignMesh->VertexCount());
        triangleCount = static_cast<int>(ignMesh->IndexCount() / 3);
        rglVertices.reserve(vertexCount);
        ignMesh->FillArrays(&ignVertices, reinterpret_cast<int**>(&triangles));
    }

    for (int i = 0; i < vertexCount; ++i) {
        rglVertices.emplace_back(rgl_vec3f{
                static_cast<float>(scaleX * ignVertices[3 * i + 0]),
                static_cast<float>(scaleY * ignVertices[3 * i + 1]),
                static_cast<float>(scaleZ * ignVertices[3 * i + 2])});
    }

    bool success = CheckRGL(rgl_mesh_create(mesh, rglVertices.data(), vertexCount, triangles, triangleCount));

    free(ignVertices);
    free(triangles);

    return success;
}

}  // namespace rgl
