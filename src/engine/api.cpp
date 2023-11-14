#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "jb_exception.h"
#include "mesh_utils.h"
#include "physics_world.h"
#include "softbody.h"

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(jellyblend_engine, m)
{
    m.doc() = "a softbody simulation library"; // optional module docstring
    m.def("softbody_mesh_from_bl_mesh", &softbody_mesh_from_bl_mesh, "mesh"_a, "max_volume"_a = -1);

    py::class_<ExportGeometry>(m, "ExportGeometry")
        .def(py::init())
        .def_readwrite("surface_vertex_num", &ExportGeometry::surface_vertex_num)
        .def_readwrite("all_vertices", &ExportGeometry::all_vertices)
        .def_readwrite("surface_faces", &ExportGeometry::surface_faces)
        .def_readwrite("all_edges", &ExportGeometry::all_edges)
        .def_readwrite("surface_edges", &ExportGeometry::surface_edges)
        .def_readwrite("tetrahedra", &ExportGeometry::tetrahedra);

    py::class_<SoftBodySetting>(m, "SoftBodySetting")
        .def(py::init())
        .def_readwrite("density", &SoftBodySetting::density)
        .def_readwrite("gravity", &SoftBodySetting::gravity)
        .def_readwrite("youngs_modulus", &SoftBodySetting::youngs_modulus)
        .def_readwrite("poissons_ratio", &SoftBodySetting::poissons_ratio)
        .def_readwrite("damping", &SoftBodySetting::damping)
        .def_readwrite("friction", &SoftBodySetting::friction)
        .def_readwrite("detect_self_collision", &SoftBodySetting::detect_self_collision);

    py::class_<PhysicsWorldSetting>(m, "PhysicsWorldSetting")
        .def(py::init())
        .def_readwrite("solver_substep_num", &PhysicsWorldSetting::solver_substep_num)
        .def_readwrite("frame_substep_num", &PhysicsWorldSetting::frame_substep_num)
        .def_readwrite("frame_rate", &PhysicsWorldSetting::frame_rate)
        .def_readwrite("passive_collision_distance", &PhysicsWorldSetting::passive_collision_distance)
        .def_readwrite("spatial_map_mem_threshold", &PhysicsWorldSetting::spatial_map_mem_threshold)
        .def_readwrite("spatial_map_size_multiplier", &PhysicsWorldSetting::spatial_map_size_multiplier);

    py::class_<PhysicsWorld>(m, "PhysicsWorld")
        .def(py::init())
        .def("dump_to_file", &PhysicsWorld::dump_to_file)
        .def("load_from_file", &PhysicsWorld::load_from_file)
        .def("add_softbody", &PhysicsWorld::add_softbody_bl)
        .def("add_fixedbody", &PhysicsWorld::add_fixedbody_bl)
        .def("prepare_simulation", &PhysicsWorld::prepare_simulation)
        .def("next_frame", &PhysicsWorld::next_frame)
        .def("simulate", &PhysicsWorld::simulate, "frame_start"_a, "frame_end"_a, "test_mode"_a = false)
        .def("load_setting", &PhysicsWorld::load_setting)
        .def("test_func", &PhysicsWorld::test_func);

    py::register_exception<MeshGenExceedIntMax>(m, "MeshGenExceedIntMax");
    py::register_exception<MeshGenOutOfMem>(m, "MeshGenOutOfMem");
    py::register_exception<MeshGenKnowBug>(m, "MeshGenKnowBug");
    py::register_exception<MeshGenSelfIntersection>(m, "MeshGenSelfIntersection");
    py::register_exception<MeshGenSmallFeature>(m, "MeshGenSmallFeature");
    py::register_exception<MeshGenUnknown>(m, "MeshGenUnknown");
    py::register_exception<BlMeshModified>(m, "BlMeshModified");
    py::register_exception<BlObjectMissing>(m, "BlObjectMissing");
    py::register_exception<SIMBlowUp>(m, "SIMBlowUp");
}