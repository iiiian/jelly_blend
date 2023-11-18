#include <Eigen/Dense>
#include <iostream>
#include <pybind11/stl.h>
#include <spdlog/spdlog.h>
#include <sstream>
#include <string>

#include "jb_exception.h"
#include "python_helper.h"
#include "softbody.h"
#include "utils.h"

void SoftBody::load_settings(const SoftBodySetting &settings)
{
    density = settings.density;
    gravity = settings.gravity;
    youngs_modulus = settings.youngs_modulus;
    poissons_ratio = settings.poissons_ratio;
    damping = settings.damping;
    friction = settings.friction;
    detect_self_collision = settings.detect_self_collision;
}

void SoftBody::load_bl_softbody_mesh(pybind11::object bl_softbody)
{
    namespace py = pybind11;

    bl_object_name = bl_softbody.attr("name").cast<std::string>();

    py::dict bl_dict = bl_softbody;
    py::object bl_mesh = bl_softbody.attr("data");

    // vertices related
    vertex_num = py_len(bl_dict["jb_softbody_mesh_initial_vertices"]) / 3;
    surface_vertex_num = bl_dict["jb_softbody_mesh_surface_vertex_num"].cast<size_t>();
    vertices.resize(3, vertex_num);
    predict_vertices.resize(3, vertex_num);
    initial_vertices.resize(3, vertex_num);
    velocity.resize(3, vertex_num);
    velocity.setZero();
    py::list bl_ini_vertices = bl_dict["jb_softbody_mesh_initial_vertices"];
    Eigen::Vector3d vertex_coor;
    for (size_t i = 0; i < vertex_num; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            vertex_coor(j) = bl_ini_vertices[3 * i + j].cast<double>();
        }
        vertices.col(i) = vertex_coor;
        initial_vertices.col(i) = vertex_coor;
    }

    edge_num = py_len(bl_dict["jb_softbody_mesh_edges"]) / 2;
    edges.resize(2, edge_num);
    py::list bl_edges = bl_dict["jb_softbody_mesh_edges"];
    for (size_t i = 0; i < edge_num; ++i)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            edges(j, i) = bl_edges[2 * i + j].cast<size_t>();
        }
    }

    surface_edge_num = py_len(bl_dict["jb_softbody_mesh_surface_edges"]) / 2;
    surface_edges.resize(2, surface_edge_num);
    py::list bl_surface_edges = bl_dict["jb_softbody_mesh_surface_edges"];
    for (size_t i = 0; i < surface_edge_num; ++i)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            surface_edges(j, i) = bl_surface_edges[2 * i + j].cast<size_t>();
        }
    }

    // surface face
    face_num = py_len(bl_dict["jb_softbody_mesh_surface_faces"]) / 3;
    faces.resize(3, face_num);
    py::list bl_face = bl_dict["jb_softbody_mesh_surface_faces"];
    for (size_t i = 0; i < face_num; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            faces(j, i) = bl_face[3 * i + j].cast<size_t>();
        }
    }

    tetra_num = py_len(bl_dict["jb_softbody_mesh_tetrahedra"]) / 4;
    tetrahedra.resize(4, tetra_num);
    py::list bl_tetra = bl_dict["jb_softbody_mesh_tetrahedra"];
    for (size_t i = 0; i < tetra_num; ++i)
    {
        for (size_t j = 0; j < 4; ++j)
        {
            tetrahedra(j, i) = bl_tetra[4 * i + j].cast<size_t>();
        }
    }
}

void SoftBody::init()
{
    construct_vertex_masses();
    construct_constant_accel();
    construct_neonewton_constrains();
}

void SoftBody::construct_vertex_masses()
{
    vertex_masses.resize(vertex_num);
    vertex_masses.setZero();

    for (size_t i = 0; i < tetra_num; ++i)
    {
        std::array<Eigen::Vector3d, 4> c; // corners

        for (size_t j = 0; j < 4; ++j)
        {
            size_t vert_index = tetrahedra(j, i);
            c[j] = initial_vertices.col(vert_index);
        }

        double volume = (c[3] - c[0]).dot((c[1] - c[0]).cross(c[2] - c[0])) / 6;
        assert((volume > 0));

        for (size_t j = 0; j < 4; ++j)
        {
            size_t vert_index = tetrahedra(j, i);
            vertex_masses(vert_index) += volume * density / 4;
        }
    }
}

void SoftBody::construct_neonewton_constrains()
{

    double lame1, lame2;

    double e = youngs_modulus;
    double v = poissons_ratio;

    lame1 = (e * v) / ((1 + v) * (1 - 2 * v));
    lame2 = e / (2 * (1 + v));

    // initial corner coordinates
    Eigen::Matrix<double, 3, 4> init_c;

    for (size_t i = 0; i < tetra_num; ++i)
    {

        auto tetra = tetrahedra.col(i);

        Eigen::Vector4d corner_weights;
        std::array<size_t, 4> corner_positions;
        for (size_t j = 0; j < 4; ++j)
        {
            corner_positions[j] = 3 * tetra[j];
            corner_weights[j] = 1 / vertex_masses[tetra[j]];
        }

        init_c.col(0) = initial_vertices.col(tetra[0]);
        init_c.col(1) = initial_vertices.col(tetra[1]);
        init_c.col(2) = initial_vertices.col(tetra[2]);
        init_c.col(3) = initial_vertices.col(tetra[3]);

        auto sp_neo = std::make_shared<NeoNewtonConstrain>(lame1, lame2, init_c, corner_weights, corner_positions);
        auto sp_soft_con = std::dynamic_pointer_cast<SoftBodyConstrain>(sp_neo);
        sp_soft_constrains.push_back(sp_soft_con);
    }
}

void SoftBody::construct_constant_accel()
{
    constant_accels.resize(3, vertex_num);
    constant_accels.setZero();
    for (size_t i = 0; i < vertex_num; ++i)
    {
        constant_accels(2, i) = -gravity;
    }
}

void SoftBody::predict(double time_delta)
{
    predict_vertices = vertices + time_delta * velocity;
    predict_vertices += 0.5 * time_delta * time_delta * constant_accels;
}

void SoftBody::update_avg_predict_edge_length()
{
    Eigen::Vector3d d;
    Eigen::Vector<size_t, 2> edge;

    // calculate average surface edge length
    avg_predict_edge_length = 0;
    for (size_t i = 0; i < surface_edge_num; ++i)
    {
        edge = surface_edges.col(i);
        avg_predict_edge_length += (predict_vertices.col(edge[0]) - predict_vertices.col(edge[1])).norm();
    }
    avg_predict_edge_length /= surface_edge_num;
}

/*
void SoftBody::insert_mesh_keyframe(int frame)
{
    namespace py = pybind11;
    using namespace pybind11::literals;

    py::object bl_vertices, bl_key_blocks, bl_shape_key, bl_shape_keys;
    std::string frame_name = "frame";
    double shape_key_internal_frame;

    py::object bl_obj = get_bl_object(bl_object_name);

    bl_vertices = bl_obj.attr("data").attr("vertices");
    if (py_len(bl_vertices) != surface_vertex_num)
    {
        throw BlMeshModified();
    }

    frame_name.append(std::to_string(frame));

    bl_shape_keys = bl_obj.attr("data").attr("shape_keys");
    // if object does not have shapekeys, create both shapekeys and shapekey
    if (bl_shape_keys.is(py::none()))
    {
        py::print("obj has no shapekey");
        bl_obj.attr("shape_key_add")("name"_a = "Base", "from_mix"_a = false);
        bl_shape_key = bl_obj.attr("shape_key_add")("name"_a = frame_name, "from_mix"_a = false);
        bl_shape_keys = bl_obj.attr("data").attr("shape_keys");
    }
    // object already have shapekeys, try to get target shapekey from shapekeys
    // if there is no target shapekey, create one
    else
    {
        bl_key_blocks = bl_shape_keys.attr("key_blocks");
        py::print("prepare to get shapekey, name:");
        py::print(frame_name);
        bl_shape_key = bl_key_blocks.attr("get")(py::cast(frame_name));
        if (bl_shape_key.is(py::none()))
        {
            py::print("no shapekey, add one, frame:");
            py::print(std::to_string(frame));
            bl_shape_key = bl_obj.attr("shape_key_add")("name"_a = py::cast(frame_name), "from_mix"_a = false);
        }
    }
    bl_shape_keys.attr("use_relative") = false;
    size_t i = 0;
    for (auto bl_shape_key_point : bl_shape_key.attr("data"))
    {
        for (size_t j = 0; j < 3; ++j)
        {
            bl_shape_key_point.attr("co").attr("__setitem__")(j, vertices(j, i));
        }
        ++i;
    }
    return;
    // insert animation
    shape_key_internal_frame = bl_shape_key.attr("frame").cast<double>();
    bl_shape_keys.attr("eval_time") = shape_key_internal_frame;
    bl_shape_keys.attr("keyframe_insert")("data_path"_a = "eval_time", "frame"_a = frame);
}
*/

std::string SoftBody::summary() const
{
    std::stringstream ss;

    ss << "-----------------SoftBody summary-----------------";
    ss << "\nAnalytics:\n";
    ss << "vertex_num = " << vertex_num << "\n";
    ss << "edge_num = " << edge_num << "\n";
    ss << "face_num = " << face_num << "\n";
    ss << "tetra_num = " << tetra_num << "\n";
    ss << "avg_surface_edge_length = " << avg_predict_edge_length << "\n";
    ss << "surface_vertex_num = " << surface_vertex_num << "\n";
    ss << "surface_edge_num = " << surface_edge_num << "\n";

    ss << "\nVertices Detail:\n";
    ss << vertices << "\n";

    ss << "\nInitial Vertices Detail:\n";
    ss << initial_vertices << "\n";

    ss << "\nPredict Vertices Detail:\n";
    ss << predict_vertices << "\n";

    ss << "\nVertices velocity Detail:\n";
    ss << velocity << "\n";

    ss << "\nEdges Detail:\n";
    ss << edges << "\n";

    ss << "\nFaces Detail:\n";
    ss << faces << "\n";

    ss << "\nTetrahedra Detail:\n";
    ss << tetrahedra << "\n";

    ss << "\nSurface Edges Detail:\n";
    ss << surface_edges << "\n";

    return ss.str();
}

SoftBodyMesh::SoftBodyMesh(const SoftBody &soft)
{
    bl_object_name = soft.bl_object_name;
    vertex_num = soft.vertex_num;
    surface_vertex_num = soft.surface_vertex_num;

    vertices.resize(3 * vertex_num);
    for (size_t i = 0; i < vertex_num; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            vertices[3 * i + j] = soft.vertices(j, i);
        }
    }
}
