#pragma once

#include <pybind11/pybind11.h>
#include <vector>

struct ExportGeometry
{
    size_t surface_vertex_num;
    std::vector<double> all_vertices;
    std::vector<size_t> surface_faces;
    std::vector<size_t> all_edges;
    std::vector<size_t> surface_edges;
    std::vector<size_t> tetrahedra;
};

ExportGeometry softbody_mesh_from_bl_mesh(pybind11::object mesh, double max_volume);