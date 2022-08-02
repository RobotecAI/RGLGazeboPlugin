#include <RGLGazeboPlugin.hh>

#include <string>

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
#define RGL_CAPSULE_TEXT "RGLGazeboPlugin_capsule_"

#define CAPSULE_RINGS 1
#define CAPSULE_SEGMENTS 32

using namespace rgl;

const ignition::common::Mesh* RGLGazeboPlugin::LoadBox(
        const ignition::gazebo::components::Geometry* geometry,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    std::cout << "BOX geometry" << std::endl;

    auto size = geometry->Data().BoxShape()->Size();

    scale_x = size.X();
    scale_y = size.Y();
    scale_z = size.Z();

    return mesh_manager->MeshByName(UNIT_BOX_TEXT);
}

const ignition::common::Mesh* RGLGazeboPlugin::LoadCapsule(const ignition::gazebo::components::Geometry* geometry) {
    std::cout << "CAPSULE geometry" << std::endl;

    static unsigned int capsule_id = 0;

    auto shape = geometry->Data().CapsuleShape();

    mesh_manager->CreateCapsule(
            RGL_CAPSULE_TEXT + std::to_string(capsule_id),
            shape->Radius(),
            shape->Length(),
            CAPSULE_RINGS,
            CAPSULE_SEGMENTS);

    capsule_id++;

    return mesh_manager->MeshByName(RGL_CAPSULE_TEXT + std::to_string(capsule_id - 1));
}

const ignition::common::Mesh* RGLGazeboPlugin::LoadCylinder(
        const ignition::gazebo::components::Geometry* geometry,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    std::cout << "CYLINDER geometry" << std::endl;

    auto shape = geometry->Data().CylinderShape();

    scale_x = shape->Radius() * 2;
    scale_y = shape->Radius() * 2;
    scale_z = shape->Length();

    return mesh_manager->MeshByName(UNIT_CYLINDER_TEXT);
}

const ignition::common::Mesh* RGLGazeboPlugin::LoadEllipsoid(
        const ignition::gazebo::components::Geometry* geometry,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    std::cout << "ELLIPSOID geometry" << std::endl;

    auto shape = geometry->Data().EllipsoidShape()->Radii();

    scale_x = shape.X() * 2;
    scale_y = shape.Y() * 2;
    scale_z = shape.Z() * 2;

    return mesh_manager->MeshByName(UNIT_SPHERE_TEXT);
}

const ignition::common::Mesh* RGLGazeboPlugin::LoadPlane(
        const ignition::gazebo::components::Geometry* geometry,
        double& scale_x,
        double& scale_y) {

    std::cout << "PLANE geometry" << std::endl;

    auto size = geometry->Data().PlaneShape()->Size();

    scale_x = size.X() * 2;
    scale_y = size.Y() * 2;

    return mesh_manager->MeshByName(UNIT_PLANE_TEXT);
}

const ignition::common::Mesh* RGLGazeboPlugin::LoadSphere(
        const ignition::gazebo::components::Geometry* geometry,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    std::cout << "SPHERE geometry" << std::endl;

    auto radius = geometry->Data().SphereShape()->Radius();

    scale_x = radius * 2;
    scale_y = radius * 2;
    scale_z = radius * 2;

    return mesh_manager->MeshByName(UNIT_SPHERE_TEXT);
}

const ignition::common::Mesh* RGLGazeboPlugin::GetMeshPointer(
        const ignition::gazebo::components::Geometry* geometry,
        double& scale_x,
        double& scale_y,
        double& scale_z) {

    const ignition::common::Mesh* mesh_pointer;
    const auto& data = geometry->Data();

    switch (data.Type()) {
        case sdf::GeometryType::BOX:
            mesh_pointer = LoadBox(geometry, scale_x, scale_y, scale_z);
            break;
        case sdf::GeometryType::CAPSULE:
            mesh_pointer = LoadCapsule(geometry);
            break;
        case sdf::GeometryType::CYLINDER:
            mesh_pointer = LoadCylinder(geometry, scale_x, scale_y, scale_z);
            break;
        case sdf::GeometryType::ELLIPSOID:
            mesh_pointer = LoadEllipsoid(geometry, scale_x, scale_y, scale_z);
            break;
        case sdf::GeometryType::EMPTY:
            std::cout << "EMPTY geometry" << std::endl;
            return nullptr;
        case sdf::GeometryType::MESH:
            std::cout << "MESH geometry" << std::endl;
            mesh_pointer = mesh_manager->MeshByName(
                    ignition::gazebo::asFullPath(
                            data.MeshShape()->Uri(),
                            data.MeshShape()->FilePath()));
            break;
        case sdf::GeometryType::PLANE:
            mesh_pointer = LoadPlane(geometry, scale_x, scale_y);
            break;
        case sdf::GeometryType::SPHERE:
            mesh_pointer = LoadSphere(geometry, scale_x, scale_y, scale_z);
            break;
        default:
            ignerr << "geometry type not supported yet" << std::endl;
            return nullptr;
    }
    assert(nullptr != mesh_pointer);
    return mesh_pointer;
}

bool RGLGazeboPlugin::GetMesh(
        const ignition::gazebo::components::Geometry* geometry,
        int& vertex_count,
        int& triangle_count,
        rgl_vec3f*& vertices,
        rgl_vec3i** triangles) {

    double scale_x = 1;
    double scale_y = 1;
    double scale_z = 1;

    auto mesh_common = GetMeshPointer(geometry, scale_x, scale_y, scale_z);
    if (nullptr == mesh_common) return false;

    vertex_count = static_cast<int>(mesh_common->VertexCount());
    // the naming scheme in rgl could be changed, because i_count is
    // a misleading name (it's equal to index_count / 3) - better use triangle_count
    triangle_count = static_cast<int>(mesh_common->IndexCount() / 3);

    vertices = static_cast<rgl_vec3f*>(malloc(sizeof(rgl_vec3f) * vertex_count));
    double* vertices_double_arr = nullptr;

    mesh_common->FillArrays(&vertices_double_arr, (int**) triangles);

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

bool RGLGazeboPlugin::LoadMeshToRGL(
        rgl_mesh_t* new_mesh,
        const ignition::gazebo::components::Geometry* geometry) {

    int vertex_count;
    int triangle_count;
    rgl_vec3f* vertices = nullptr;
    rgl_vec3i* triangles = nullptr;

    if (!GetMesh(geometry, vertex_count, triangle_count, vertices, &triangles)) return false;
    RGL_CHECK(rgl_mesh_create(new_mesh, vertices, vertex_count, triangles, triangle_count));

    free(vertices);
    free(triangles);

    return true;
}