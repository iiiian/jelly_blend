#include <pybind11/pybind11.h>
#include <sstream>

#include "fixedbody.h"
#include "jb_exception.h"
#include "python_helper.h"

FixedBody::FixedBody(pybind11::object bl_fixedbody)
{
    namespace py = pybind11;
    py::object bl_types_object = py::module_::import("bpy").attr("types").attr("Object");
    size_t i, j;

    bl_object_name = bl_fixedbody.attr("name").cast<std::string>();

    BlEvalMesh helper_mesh(bl_fixedbody);
    py::object bl_mesh = helper_mesh.bl_mesh;

    vertex_num = py_len(bl_mesh.attr("vertices"));
    edge_num = py_len(bl_mesh.attr("edges"));
    face_num = py_len(bl_mesh.attr("polygons"));

    vertices.resize(3, vertex_num);
    velocity.resize(3, vertex_num);
    predict_vertices.resize(3, vertex_num);
    edges.resize(2, edge_num);
    faces.resize(3, face_num);
    prev_frame_vertices.resize(3, vertex_num);
    next_frame_vertices.resize(3, vertex_num);

    velocity.setZero();

    BlVertices helper_vertices(bl_mesh.attr("vertices"));
    helper_vertices.copy_to(vertices);

    i = 0;
    for (auto bl_edge : bl_mesh.attr("edges"))
    {
        j = 0;
        for (auto bl_vertex_index : bl_edge.attr("vertices"))
        {
            edges(j, i) = bl_vertex_index.cast<size_t>();
            ++j;
        }
        ++i;
    }

    i = 0;
    for (auto bl_poly : bl_mesh.attr("polygons"))
    {
        j = 0;
        for (auto bl_vertex_index : bl_poly.attr("vertices"))
        {
            faces(j, i) = bl_vertex_index.cast<size_t>();
            ++j;
        }
        ++i;
    }

    // py::print(summary());
}

double FixedBody::get_predict_edge_length_sum() const
{
    // calculate average surface edge length
    double sum = 0;
    for (auto edge : edges.colwise())
    {
        sum += (predict_vertices.col(edge(0)) - predict_vertices.col(edge(1))).norm();
    }

    return sum;
}

void FixedBody::update_frame_vert(int next_frame)
{
    namespace py = pybind11;

    py::object bl_obj = get_bl_object(bl_object_name);

    prev_frame_vertices = next_frame_vertices;
    if (next_frame_vertices.cols() != vertex_num)
    {
        next_frame_vertices.resize(3, vertex_num);
    }

    BlEvalMesh helper_mesh(bl_obj, next_frame);
    py::object bl_mesh = helper_mesh.bl_mesh;

    BlVertices helper_vertices(bl_mesh.attr("vertices"));
    helper_vertices.copy_to(next_frame_vertices, true);
}

void FixedBody::predict(double progress, bool test_mode)
{

    if (test_mode)
    {
        predict_vertices = vertices;
        return;
    }

    assert((progress >= 0 && progress <= 1));
    assert((predict_vertices.cols() == prev_frame_vertices.cols() &&
            predict_vertices.cols() == next_frame_vertices.cols()));

    predict_vertices = (1 - progress) * prev_frame_vertices + progress * next_frame_vertices;
}

std::string FixedBody::summary() const
{
    std::stringstream ss;

    ss << "-----------------FixedBody summary-----------------";
    ss << "\nAnalytics:\n";
    ss << "vertex_num = " << vertex_num << "\n";
    ss << "edge_num = " << edge_num << "\n";
    ss << "face_num = " << face_num << "\n";

    ss << "\nVertices Detail:\n";
    ss << vertices << "\n";

    ss << "\nPredict Vertices Detail:\n";
    ss << predict_vertices << "\n";

    ss << "\nEdges Detail:\n";
    ss << edges << "\n";

    ss << "\nFaces Detail:\n";
    ss << faces << "\n";

    ss << "\nPrev frame vertices"
       << "\n";
    ss << prev_frame_vertices << "\n";

    ss << "\nNext frame vertices"
       << "\n";
    ss << prev_frame_vertices << "\n";

    return ss.str();
}