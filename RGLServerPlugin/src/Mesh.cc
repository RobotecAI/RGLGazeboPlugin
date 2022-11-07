#include "RGLServerPluginManager.hh"

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
#define RGL_UNIT_CAPSULE_TEXT "RGLGazeboPlugin_unit_capsule"

#define CAPSULE_RINGS 1
#define CAPSULE_SEGMENTS 32

using namespace rgl;

const ignition::common::Mesh* RGLServerPluginManager::LoadBox(
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

const ignition::common::Mesh* RGLServerPluginManager::LoadCapsule(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    static bool unit_capsule_mesh_created = false;
    if (!unit_capsule_mesh_created) {
        mesh_manager->CreateCapsule(
                RGL_UNIT_CAPSULE_TEXT,
                0.5f,
                1.0f,
                CAPSULE_RINGS,
                CAPSULE_SEGMENTS);
        unit_capsule_mesh_created = true;
    }

    auto shape = data.CapsuleShape();

    scale_x = shape->Radius() * 2;
    scale_y = shape->Radius() * 2;
    scale_z = shape->Length();

    return mesh_manager->MeshByName(RGL_UNIT_CAPSULE_TEXT);
}

const ignition::common::Mesh* RGLServerPluginManager::LoadCylinder(
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

const ignition::common::Mesh* RGLServerPluginManager::LoadEllipsoid(
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

const ignition::common::Mesh* RGLServerPluginManager::LoadMesh(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    auto scale = data.MeshShape()->Scale();

    scale_x = scale.X();
    scale_y = scale.Y();
    scale_z = scale.Z();

    return mesh_manager->MeshByName(
            ignition::gazebo::asFullPath(
                    data.MeshShape()->Uri(),
                    data.MeshShape()->FilePath()));
}

const ignition::common::Mesh* RGLServerPluginManager::LoadPlane(
        const sdf::Geometry& data,
        double& scale_x,
        double& scale_y) {

    auto size = data.PlaneShape()->Size();

    scale_x = size.X() * 2;
    scale_y = size.Y() * 2;

    return mesh_manager->MeshByName(UNIT_PLANE_TEXT);
}

const ignition::common::Mesh* RGLServerPluginManager::LoadSphere(
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

const ignition::common::Mesh* RGLServerPluginManager::GetMeshPointer(
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
            mesh_pointer = LoadCapsule(data, scale_x, scale_y, scale_z);
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
            ignerr << "geometry type: " << (int)data.Type() << " not supported yet" << std::endl;
            return nullptr;
    }
    if (nullptr == mesh_pointer) {
        ignerr << "Error in importing mesh - it will not be loaded to rgl\n";
    }
    return mesh_pointer;
}

bool RGLServerPluginManager::LoadMeshToRGL(
        rgl_mesh_t* new_mesh,
        const sdf::Geometry& data) {

    double scale_x = 1;
    double scale_y = 1;
    double scale_z = 1;

    auto mesh_common = GetMeshPointer(data, scale_x, scale_y, scale_z);
    if (nullptr == mesh_common) {
        return false;
    }

    int vertex_count = (int)mesh_common->VertexCount();
    int triangle_count = (int)mesh_common->IndexCount() / 3;
    std::vector<rgl_vec3f> vertices;
    std::vector<rgl_vec3i> triangles;

    vertices.resize(vertex_count);

    double* vertices_double_arr = nullptr;
    int* triangles_arr = nullptr;

    mesh_common->FillArrays(&vertices_double_arr, &triangles_arr);

    auto* beg = reinterpret_cast<rgl_vec3i*>(triangles_arr);
    auto* end = reinterpret_cast<rgl_vec3i*>(triangles_arr + sizeof(rgl_vec3i) * triangle_count);
    triangles.assign(beg, end);

    for (int i = 0; i < vertex_count; ++i) {
        vertices[i].value[0] = RoundFloat(static_cast<float>(scale_x * vertices_double_arr[3 * i + 0]));
        vertices[i].value[1] = RoundFloat(static_cast<float>(scale_y * vertices_double_arr[3 * i + 1]));
        vertices[i].value[2] = RoundFloat(static_cast<float>(scale_z * vertices_double_arr[3 * i + 2]));
    }

    free(vertices_double_arr);

    RGL_CHECK(rgl_mesh_create(new_mesh, vertices.data(), vertex_count, triangles.data(), triangle_count));

    return true;
}
