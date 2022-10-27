#include "RGLServerPlugin.hh"

#include <ignition/common/Mesh.hh>
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
#define RGL_CAPSULE_TEXT "RGLGazeboPlugin_capsule_"

#define CAPSULE_RINGS 1
#define CAPSULE_SEGMENTS 32

using namespace rgl;

const ignition::common::Mesh* RGLServerPlugin::LoadBox(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    std::cout << "BOX geometry" << std::endl;

    auto size = data.BoxShape()->Size();

    scale_x = size.X();
    scale_y = size.Y();
    scale_z = size.Z();

    return mesh_manager->MeshByName(UNIT_BOX_TEXT);
}

const ignition::common::Mesh* RGLServerPlugin::LoadCapsule(const sdf::Geometry& data) {
    std::cout << "CAPSULE geometry" << std::endl;

    static unsigned int capsule_id = 0;

    auto shape = data.CapsuleShape();

    mesh_manager->CreateCapsule(
            RGL_CAPSULE_TEXT + std::to_string(capsule_id),
            shape->Radius(),
            shape->Length(),
            CAPSULE_RINGS,
            CAPSULE_SEGMENTS);

    capsule_id++;

    return mesh_manager->MeshByName(RGL_CAPSULE_TEXT + std::to_string(capsule_id - 1));
}

const ignition::common::Mesh* RGLServerPlugin::LoadCylinder(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    std::cout << "CYLINDER geometry" << std::endl;

    auto shape = data.CylinderShape();

    scale_x = shape->Radius() * 2;
    scale_y = shape->Radius() * 2;
    scale_z = shape->Length();

    return mesh_manager->MeshByName(UNIT_CYLINDER_TEXT);
}

const ignition::common::Mesh* RGLServerPlugin::LoadEllipsoid(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    std::cout << "ELLIPSOID geometry" << std::endl;

    auto shape = data.EllipsoidShape()->Radii();

    scale_x = shape.X() * 2;
    scale_y = shape.Y() * 2;
    scale_z = shape.Z() * 2;

    return mesh_manager->MeshByName(UNIT_SPHERE_TEXT);
}

const ignition::common::Mesh* RGLServerPlugin::LoadMesh(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    std::cout << "MESH geometry" << std::endl;

    auto scale = data.MeshShape()->Scale();

    scale_x = scale.X();
    scale_y = scale.Y();
    scale_z = scale.Z();

    return mesh_manager->MeshByName(
            ignition::gazebo::asFullPath(
                    data.MeshShape()->Uri(),
                    data.MeshShape()->FilePath()));
}

const ignition::common::Mesh* RGLServerPlugin::LoadPlane(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y) {

    std::cout << "PLANE geometry" << std::endl;

    auto size = data.PlaneShape()->Size();

    scale_x = size.X() * 2;
    scale_y = size.Y() * 2;

    return mesh_manager->MeshByName(UNIT_PLANE_TEXT);
}

const ignition::common::Mesh* RGLServerPlugin::LoadSphere(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    std::cout << "SPHERE geometry" << std::endl;

    auto radius = data.SphereShape()->Radius();

    scale_x = radius * 2;
    scale_y = radius * 2;
    scale_z = radius * 2;

    return mesh_manager->MeshByName(UNIT_SPHERE_TEXT);
}

const ignition::common::Mesh* RGLServerPlugin::GetMeshPointer(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    const ignition::common::Mesh* mesh_pointer;

    switch (data.Type()) {
        case sdf::GeometryType::BOX:
            mesh_pointer = LoadBox(data, scale_x, scale_y, scale_z);
            break;
        case sdf::GeometryType::CAPSULE:
            mesh_pointer = LoadCapsule(data);
            break;
        case sdf::GeometryType::CYLINDER:
            mesh_pointer = LoadCylinder(data, scale_x, scale_y, scale_z);
            break;
        case sdf::GeometryType::ELLIPSOID:
            mesh_pointer = LoadEllipsoid(data, scale_x, scale_y, scale_z);
            break;
        case sdf::GeometryType::EMPTY:
            ignerr << "EMPTY geometry" << std::endl;
            return nullptr;
        case sdf::GeometryType::MESH:
            mesh_pointer = LoadMesh(data, scale_x, scale_y, scale_z);
            break;
        case sdf::GeometryType::PLANE:
            mesh_pointer = LoadPlane(data, scale_x, scale_y);
            break;
        case sdf::GeometryType::SPHERE:
            mesh_pointer = LoadSphere(data, scale_x, scale_y, scale_z);
            break;
        default:
            ignerr << "geometry type not supported yet" << std::endl;
            return nullptr;
    }
    if (nullptr == mesh_pointer) {
        std::cout << "Error in importing mesh - it will not be loaded to RGL\n";
    }
    return mesh_pointer;
}

bool RGLServerPlugin::GetMesh(
        const sdf::Geometry& data,
        size_t& vertex_count,
        size_t& triangle_count,
        std::vector<rgl_vec3f>& vertices,
        std::vector<rgl_vec3i>& triangles) {

    double scale_x = 1;
    double scale_y = 1;
    double scale_z = 1;

    auto mesh_common = GetMeshPointer(data, scale_x, scale_y, scale_z);
    if (nullptr == mesh_common) return false;

    vertex_count = mesh_common->VertexCount();
    triangle_count = mesh_common->IndexCount() / 3;

    vertices.resize(vertex_count);

    double* vertices_double_arr = nullptr;
    int* triangles_arr = nullptr;

    mesh_common->FillArrays(&vertices_double_arr, &triangles_arr);

    auto* beginning = reinterpret_cast<rgl_vec3i*>(triangles_arr);
    auto* end = reinterpret_cast<rgl_vec3i*>(triangles_arr + sizeof(rgl_vec3i) * triangle_count);
    triangles.assign(beginning, end);

    int v_index = 0;
    for (int i = 0; i < vertex_count; ++i) {
        for (int j = 0; j < 3; ++j) {
            auto vertex_coord = vertices_double_arr[v_index];
            switch (j) {
                case 0:
                    vertex_coord *= scale_x;
                    break;
                case 1:
                    vertex_coord *= scale_y;
                    break;
                case 2:
                    vertex_coord *= scale_z;
                    break;
                default:;
            }
            vertices[i].value[j] = RoundFloat(static_cast<float>(vertex_coord));
            v_index++;
        }
    }

    /// Debug printf
//    int count = 0;
//    std::cout << "vertices: ";
//    for (int i = 0; i < vertex_count; ++i) {
//        std::cout << count << ": ";
//        count++;
//        for (int j = 0; j < 3; ++j) {
//            std::cout << vertices[i].value[j] << ",";
//        }
//        std::cout << " ";
//    }
//    std::cout << "\n";
//
//    std::cout << "triangles: ";
//    for (int i = 0; i < triangle_count; ++i) {
//        for (int j = 0; j < 3; ++j) {
//            std::cout << (*triangles)[i].value[j] << ",";
//        }
//        std::cout << " ";
//    }
//    std::cout << "\n";

    free(vertices_double_arr);
    return true;
}

bool RGLServerPlugin::LoadMeshToRGL(
        rgl_mesh_t* new_mesh,
        const sdf::Geometry& data) {

    size_t vertex_count;
    size_t triangle_count;
    std::vector<rgl_vec3f> vertices;
    std::vector<rgl_vec3i> triangles;

    if (!GetMesh(data, vertex_count, triangle_count, vertices, triangles)) return false;
    RGL_CHECK(rgl_mesh_create(new_mesh, vertices.data(), vertex_count, triangles.data(), triangle_count));

    return true;
}
