#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <spdlog/spdlog.h>
#include <sstream>

#include "jb_exception.h"
#include "python_helper.h"

namespace py = pybind11;
using namespace pybind11::literals;

size_t py_len(pybind11::object obj)
{
    py::object len = py::module_::import("builtins").attr("len");
    return len(obj).cast<size_t>();
}

pybind11::object get_bl_object(std::string name)
{
    py::dict bl_data_objects = py::module_::import("bpy").attr("data").attr("objects");

    py::object bl_obj = bl_data_objects.attr("get")(py::cast(name));
    if (bl_obj.is(py::none()))
    {
        throw BlObjectMissing();
    }

    return bl_obj;
}

void bl_frame_set(int frame)
{
    py::object bl_scene = py::module_::import("bpy").attr("context").attr("scene");
    if (!bl_scene.is(py::none()))
    {
        bl_scene.attr("frame_set")(frame);
    }
}

BlEvalMesh::BlEvalMesh(pybind11::object bl_obj, int frame)
{
    py::object bl_context = py::module_::import("bpy").attr("context");
    py::object bl_scene = bl_context.attr("scene");

    if (frame != -1)
    {
        assert((frame > 0));
        bl_scene.attr("frame_set")(py::cast(frame));
    }

    py::object bl_depsgraph = bl_context.attr("evaluated_depsgraph_get")();
    bl_obj_eval = bl_obj.attr("evaluated_get")(bl_depsgraph);
    bl_mesh = bl_obj_eval.attr("to_mesh")();
    bl_mesh.attr("transform")(bl_obj_eval.attr("matrix_world"));
}

BlEvalMesh::~BlEvalMesh()
{
    bl_obj_eval.attr("to_mesh_clear")();
}

std::string BlEvalMesh::summary()
{
    std::stringstream ss;

    ss << "-----------------BlEvalMesh summary-----------------";
    ss << "\nAnalytics:\n";
    ss << "bl name = " << bl_obj_eval.attr("name").cast<std::string>() << "\n";
    ss << "mesh vert num = " << py_len(bl_mesh.attr("vertices")) << "\n";
    ss << "mesh edge num = " << py_len(bl_mesh.attr("edges")) << "\n";
    ss << "mesh face num = " << py_len(bl_mesh.attr("polygons")) << "\n";

    return ss.str();
}

BlVertices::BlVertices(pybind11::object bl_vertices) : bl_vertices(bl_vertices)
{
}

void BlVertices::copy_to(VERTICES &target_vertices, bool preserve_vert_num)
{

    size_t vert_num = py_len(bl_vertices);

    if (preserve_vert_num && vert_num != target_vertices.cols())
    {
        throw BlMeshModified();
    }

    target_vertices.resize(3, vert_num);

    for (size_t i = 0; i < vert_num; ++i)
    {
        py::object bl_vertex = bl_vertices.attr("__getitem__")(i);

        for (size_t j = 0; j < 3; ++j)
        {
            target_vertices(j, i) = bl_vertex.attr("co").attr("__getitem__")(j).cast<double>();
        }
    }
}

void BlTransformer::load_transformer(pybind11::object obj)
{
    py::object bl_location = obj.attr("location");
    for (size_t i = 0; i < 3; ++i)
    {
        translation(i) = bl_location.attr("__getitem__")(i).cast<double>();
    }

    py::object bl_scale = obj.attr("scale");
    for (size_t i = 0; i < 3; ++i)
    {
        scale(i) = bl_scale.attr("__getitem__")(i).cast<double>();
    }

    py::object bl_matrix_world = obj.attr("matrix_world");
    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            rotation(i, j) = bl_matrix_world.attr("__getitem__")(i).attr("__getitem__")(j).cast<double>();
        }
    }
}

Eigen::Vector3d BlTransformer::to_global(Eigen::Ref<const Eigen::Vector3d> vec)
{
    Eigen::Vector3d result;

    result = vec.array() * scale.array();
    result = (rotation * result).eval();
    result += translation;

    return result;
}

void BlTransformer::to_global_inplace(Eigen::Ref<Eigen::Vector3d> vec)
{

    vec = vec.array() * scale.array();
    vec += translation;
    vec = (rotation * vec).eval();
}

Eigen::Vector3d BlTransformer::to_local(Eigen::Ref<const Eigen::Vector3d> vec)
{
    Eigen::Vector3d result;

    result = vec - translation;
    result = (rotation.transpose() * result).eval();
    result = result.array() / scale.array();

    return result;
}

void BlTransformer::to_local_inplace(Eigen::Ref<Eigen::Vector3d> vec)
{

    vec = vec.array() / scale.array();
    vec -= translation;
    vec = (rotation.transpose() * vec).eval();
}

std::string BlTransformer::summary()
{
    std::stringstream ss;

    ss << "-----------------BlTransformer summary-----------------\n";
    ss << "translation = " << translation.transpose() << "\n";
    ss << "scale = " << scale.transpose() << "\n";
    ss << "rotation:\n";
    ss << rotation << "\n";

    return ss.str();
}
