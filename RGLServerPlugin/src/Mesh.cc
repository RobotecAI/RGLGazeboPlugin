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

#include "RGLServerPluginManager.hh"

#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>
#include <ignition/gazebo/Util.hh>

#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

#define UNIT_BOX_TEXT "unit_box"
#define UNIT_CYLINDER_TEXT "unit_cylinder"
#define UNIT_PLANE_TEXT "unit_plane"
#define UNIT_SPHERE_TEXT "unit_sphere"
//#define RGL_UNIT_CAPSULE_TEXT "RGLGazeboPlugin_unit_capsule"

//#define CAPSULE_RINGS 100
//#define CAPSULE_SEGMENTS 128

using namespace rgl;

MeshInfo RGLServerPluginManager::LoadBox(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    auto size = data.BoxShape()->Size();

    scale_x = size.X();
    scale_y = size.Y();
    scale_z = size.Z();

    return mesh_manager->MeshByName(UNIT_BOX_TEXT);
}

// Currently disabled due to being untrue with the sim gui, find out default values and reimplement

//The difference between capsule and other primitive geometry types is
// that "unit_capsule" mesh is not created by the gazebo MeshManager,
// while other primitive types have their unit meshes created and ready to query by name
//const ignition::common::Mesh* RGLServerPluginManager::LoadCapsule(
//        const sdf::Geometry& data,
//        double& scale_x,
//        double& scale_y,
//        double& scale_z) {
//
//    static bool unit_capsule_mesh_created = false;
//    if (!unit_capsule_mesh_created) {
//        mesh_manager->CreateCapsule(
//                RGL_UNIT_CAPSULE_TEXT,
//                0.5f,
//                1.0f,
//                CAPSULE_RINGS,
//                CAPSULE_SEGMENTS);
//        unit_capsule_mesh_created = true;
//    }
//
//    auto shape = data.CapsuleShape();
//
//    scale_x = shape->Radius() * 2;
//    scale_y = shape->Radius() * 2;
//    scale_z = shape->Length();
//
//    return mesh_manager->MeshByName(RGL_UNIT_CAPSULE_TEXT);
//}

MeshInfo RGLServerPluginManager::LoadCylinder(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    auto shape = data.CylinderShape();

    scale_x = shape->Radius() * 2;
    scale_y = shape->Radius() * 2;
    scale_z = shape->Length();

    return mesh_manager->MeshByName(UNIT_CYLINDER_TEXT);
}

MeshInfo RGLServerPluginManager::LoadEllipsoid(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    auto shape = data.EllipsoidShape()->Radii();

    scale_x = shape.X() * 2;
    scale_y = shape.Y() * 2;
    scale_z = shape.Z() * 2;

    return mesh_manager->MeshByName(UNIT_SPHERE_TEXT);
}

MeshInfo RGLServerPluginManager::LoadMesh(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    auto scale = data.MeshShape()->Scale();

    scale_x = scale.X();
    scale_y = scale.Y();
    scale_z = scale.Z();

    auto mesh = mesh_manager->Load(ignition::gazebo::asFullPath(
            data.MeshShape()->Uri(),
            data.MeshShape()->FilePath()));

    auto submesh_name = data.MeshShape()->Submesh();
    if (!submesh_name.empty()) {
        ignition::common::SubMesh submesh_copy(*(mesh->SubMeshByName(submesh_name).lock()));
        if (data.MeshShape()->CenterSubmesh()) {
            submesh_copy.Center();
        }
        return submesh_copy;
    } else {
        return mesh;
    }
}

MeshInfo RGLServerPluginManager::LoadPlane(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y) {

    auto size = data.PlaneShape()->Size();

    scale_x = size.X() * 2;
    scale_y = size.Y() * 2;

    return mesh_manager->MeshByName(UNIT_PLANE_TEXT);
}

MeshInfo RGLServerPluginManager::LoadSphere(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    auto radius = data.SphereShape()->Radius();

    scale_x = radius * 2;
    scale_y = radius * 2;
    scale_z = radius * 2;

    return mesh_manager->MeshByName(UNIT_SPHERE_TEXT);
}

MeshInfo RGLServerPluginManager::GetMeshPointer(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    MeshInfo mesh_info;

    switch (data.Type()) {
        case sdf::GeometryType::BOX:
            mesh_info = LoadBox(data, scale_x, scale_y, scale_z);
            break;
//        case sdf::GeometryType::CAPSULE:
//            mesh_info = LoadCapsule(data, scale_x, scale_y, scale_z);
//            break;
        case sdf::GeometryType::CYLINDER:
            mesh_info = LoadCylinder(data, scale_x, scale_y, scale_z);
            break;
        case sdf::GeometryType::ELLIPSOID:
            mesh_info = LoadEllipsoid(data, scale_x, scale_y, scale_z);
            break;
        case sdf::GeometryType::EMPTY:
            ignerr << "EMPTY geometry" << std::endl;
            return {};
        case sdf::GeometryType::MESH:
            mesh_info = LoadMesh(data, scale_x, scale_y, scale_z);
            break;
        case sdf::GeometryType::PLANE:
            mesh_info = LoadPlane(data, scale_x, scale_y);
            break;
        case sdf::GeometryType::SPHERE:
            mesh_info = LoadSphere(data, scale_x, scale_y, scale_z);
            break;
        default:
            ignerr << "geometry type: " << static_cast<int>(data.Type()) << " not supported yet" << std::endl;
            return {};
    }
    if (std::get_if<const ignition::common::Mesh*>(&mesh_info) == nullptr &&
        std::get_if<ignition::common::SubMesh>(&mesh_info) == nullptr) {
        if (data.Type() == sdf::GeometryType::MESH) {
            ignerr << "Error in importing mesh: " <<
            ignition::gazebo::asFullPath(
                    data.MeshShape()->Uri(),
                    data.MeshShape()->FilePath())
            << " - it will not be loaded to RGL\n";
        } else {
            ignerr << "Error in importing mesh - it will not be loaded to RGL\n";
        }
    }
    return mesh_info;
}

bool RGLServerPluginManager::LoadMeshToRGL(
        rgl_mesh_t* new_mesh,
        const sdf::Geometry& data) {

    double scale_x = 1;
    double scale_y = 1;
    double scale_z = 1;

    auto mesh_info = GetMeshPointer(data, scale_x, scale_y, scale_z);
    if (std::get_if<const ignition::common::Mesh*>(&mesh_info) == nullptr &&
        std::get_if<ignition::common::SubMesh>(&mesh_info) == nullptr) {
        return false;
    }

    int vertex_count;
    int triangle_count;
    std::vector<rgl_vec3f> vertices;
    double* vertices_double_arr = nullptr;
    rgl_vec3i* triangles = nullptr;

    if (std::holds_alternative<ignition::common::SubMesh>(mesh_info)) {
        auto submesh = get<ignition::common::SubMesh>(mesh_info);
        vertex_count = static_cast<int>(submesh.VertexCount());
        triangle_count = static_cast<int>(submesh.IndexCount() / 3);
        vertices.reserve(vertex_count);
        submesh.FillArrays(&vertices_double_arr, reinterpret_cast<int**>(&triangles));
    } else {
        auto mesh = get<const ignition::common::Mesh*>(mesh_info);
        vertex_count = static_cast<int>(mesh->VertexCount());
        triangle_count = static_cast<int>(mesh->IndexCount() / 3);
        vertices.reserve(vertex_count);
        mesh->FillArrays(&vertices_double_arr, reinterpret_cast<int**>(&triangles));
    }

    for (int i = 0; i < vertex_count; ++i) {
        vertices.emplace_back(rgl_vec3f{
                static_cast<float>(scale_x * vertices_double_arr[3 * i + 0]),
                static_cast<float>(scale_y * vertices_double_arr[3 * i + 1]),
                static_cast<float>(scale_z * vertices_double_arr[3 * i + 2])});
    }

    RGL_CHECK(rgl_mesh_create(new_mesh, vertices.data(), vertex_count, triangles, triangle_count));

    free(vertices_double_arr);
    free(triangles);

    return true;
}
