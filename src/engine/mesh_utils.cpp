#include <array>
#include <pybind11/pybind11.h>
#include <string>
#include <unordered_set>
#include <vector>

#include "jb_exception.h"
#include "mesh_utils.h"
#include "python_helper.h"

#define TETLIBRARY
#include "tetgen.h"

namespace py = pybind11;
using namespace pybind11::literals;

// convert tetgenio data to stdv vectors
// rearrange vertices so surface vertex is at the front
// correct the normal of faces
// return the number of surface vertices
ExportGeometry process_tetgenio(const tetgenio *data)
{

    size_t face_num = data->numberoftrifaces;
    size_t vertex_num = data->numberofpoints;
    size_t tetra_num = data->numberoftetrahedra;

    std::vector<bool> is_vertex_on_surface;
    is_vertex_on_surface.resize(vertex_num, false);
    for (size_t i = 0; i < face_num; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            is_vertex_on_surface[data->trifacelist[3 * i + j]] = true;
        }
    }

    ExportGeometry geometry;
    geometry.all_vertices.resize(3 * vertex_num);
    geometry.surface_faces.resize(3 * face_num);
    geometry.tetrahedra.resize(4 * tetra_num);

    std::vector<size_t> new_index_map;
    new_index_map.resize(vertex_num);

    // surface vertex
    size_t surface_vertex_num = 0;
    for (size_t i = 0; i < vertex_num; ++i)
    {
        if (!is_vertex_on_surface[i])
        {
            continue;
        }

        for (size_t j = 0; j < 3; ++j)
        {
            geometry.all_vertices[3 * surface_vertex_num + j] = data->pointlist[3 * i + j];
        }
        new_index_map[i] = surface_vertex_num;
        surface_vertex_num++;
    }
    // interior vertex
    size_t interior_vertex_num = 0;
    for (size_t i = 0; i < vertex_num; ++i)
    {
        if (is_vertex_on_surface[i])
        {
            continue;
        }

        for (size_t j = 0; j < 3; ++j)
        {
            geometry.all_vertices[3 * (surface_vertex_num + interior_vertex_num) + j] = data->pointlist[3 * i + j];
        }
        new_index_map[i] = surface_vertex_num + interior_vertex_num;
        interior_vertex_num++;
    }

    for (size_t i = 0; i < face_num; ++i)
    {
        // correct the normal of the face
        geometry.surface_faces[3 * i + 0] = new_index_map[data->trifacelist[3 * i + 1]];
        geometry.surface_faces[3 * i + 1] = new_index_map[data->trifacelist[3 * i + 0]];
        geometry.surface_faces[3 * i + 2] = new_index_map[data->trifacelist[3 * i + 2]];
    }

    for (size_t i = 0; i < tetra_num; ++i)
    {
        for (size_t j = 0; j < 4; ++j)
        {
            geometry.tetrahedra[4 * i + j] = new_index_map[data->tetrahedronlist[4 * i + j]];
        }
    }
    geometry.surface_vertex_num = surface_vertex_num;

    return geometry;
}

struct IdxEdge
{
    size_t p0;
    size_t p1;

    IdxEdge(size_t p0, size_t p1) : p0(p0), p1(p1){};
};

inline bool operator==(IdxEdge const &e0, IdxEdge const &e1)
{
    return (e0.p0 == e1.p0 && e0.p1 == e1.p1) || (e0.p0 == e1.p1 && e0.p1 == e1.p0);
}

// 1. vertices of a edge must be different, no risk of hashing to 0
// 2. order of vertices does not matters here, so it's fine that xor is
//    symmetric
struct IdxEdgeHash
{
  public:
    std::size_t operator()(const IdxEdge &e) const
    {
        return std::hash<size_t>()(e.p0) ^ std::hash<size_t>()(e.p1);
    }
};

void find_all_edges(ExportGeometry &geometry)
{
    std::unordered_set<IdxEdge, IdxEdgeHash> edge_set;
    size_t tetra_num = geometry.tetrahedra.size() / 4;
    for (size_t tetra_index = 0; tetra_index < tetra_num; ++tetra_index)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            for (size_t j = i + 1; j < 4; ++j)
            {
                size_t p1_index = geometry.tetrahedra[4 * tetra_index + i];
                size_t p2_index = geometry.tetrahedra[4 * tetra_index + j];
                edge_set.emplace(p1_index, p2_index);
            }
        }
    }
    size_t edge_num = edge_set.size();
    geometry.all_edges.resize(edge_num * 2);
    size_t i = 0; // index of edge
    for (const IdxEdge &edge : edge_set)
    {
        geometry.all_edges[2 * i] = edge.p0;
        geometry.all_edges[2 * i + 1] = edge.p1;
        i++;
    }
}

void find_all_surface_edges(ExportGeometry &geometry)
{
    std::unordered_set<IdxEdge, IdxEdgeHash> edge_set;
    size_t face_num = geometry.surface_faces.size() / 3;
    for (size_t i = 0; i < face_num; ++i)
    {
        size_t p1 = geometry.surface_faces[3 * i];
        size_t p2 = geometry.surface_faces[3 * i + 1];
        size_t p3 = geometry.surface_faces[3 * i + 2];

        edge_set.emplace(p1, p2);
        edge_set.emplace(p2, p3);
        edge_set.emplace(p1, p3);
    }
    size_t surface_edge_num = edge_set.size();
    geometry.surface_edges.resize(surface_edge_num * 2);
    size_t i = 0; // index of edge
    for (const IdxEdge &edge : edge_set)
    {
        geometry.surface_edges[2 * i] = edge.p0;
        geometry.surface_edges[2 * i + 1] = edge.p1;
        i++;
    }
}

ExportGeometry softbody_mesh_from_bl_mesh(pybind11::object mesh, double max_volume)
{
    size_t i, j;

    // vertices
    py::object bl_vertices = mesh.attr("vertices");
    if (py_len(bl_vertices) > INT_MAX / 3)
    {
        throw MeshGenExceedIntMax();
    }

    tetgenio in;
    in.numberofpoints = (int)py_len(bl_vertices);
    in.pointlist = new REAL[in.numberofpoints * 3];

    i = 0;
    for (auto bl_vertex : bl_vertices)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            py::float_ val = bl_vertex.attr("co").attr("__getitem__")(j);
            in.pointlist[3 * i + j] = val.cast<double>();
        }
        ++i;
    }

    // polygons
    py::object bl_polygons = mesh.attr("polygons");
    in.numberoffacets = (int)py_len(bl_polygons);
    in.facetlist = new tetgenio::facet[in.numberoffacets];
    // tetgen will set them to -1
    in.facetmarkerlist = new int[in.numberoffacets];

    i = 0;
    tetgenio::facet *pfacet;
    tetgenio::polygon *ppolygon;
    for (auto bl_polygon : bl_polygons)
    {
        in.facetmarkerlist[i] = 1;
        pfacet = &in.facetlist[i];
        pfacet->numberofpolygons = 1;
        pfacet->polygonlist = new tetgenio::polygon[pfacet->numberofpolygons];
        pfacet->numberofholes = 0;
        pfacet->holelist = NULL;
        ppolygon = &pfacet->polygonlist[0];
        ppolygon->numberofvertices = py_len(bl_polygon.attr("vertices"));
        ppolygon->vertexlist = new int[ppolygon->numberofvertices];

        j = 0;
        for (auto bl_vertex_index : bl_polygon.attr("vertices"))
        {
            ppolygon->vertexlist[j] = bl_vertex_index.cast<int>();
            j++;
        }
        i++;

        // TODO: polygon check
    }

    // spdlog::info("Mesh in:");
    // spdlog::info("  vertex num = {}", in.numberofpoints);
    // spdlog::info("  poly num = {}", in.numberoffacets);

    tetgenio out;
    std::string tet_switch = "pq1.5QY";
    if (max_volume > 0)
    {
        tet_switch.append("a");
        tet_switch.append(std::to_string(max_volume));
    }
    char *sw = const_cast<char *>(tet_switch.c_str());

    try
    {
        tetrahedralize(sw, &in, &out);
    }
    catch (int e)
    {
        switch (e)
        {
        case 1:
            throw MeshGenOutOfMem();
            break;
        case 2:
            throw MeshGenKnowBug();
            break;
        case 3:
            throw MeshGenSelfIntersection();
            break;
        case 4:
            throw MeshGenSmallFeature();
            break;
        default:
            throw MeshGenUnknown();
        }
    }

    ExportGeometry geometry;
    geometry = process_tetgenio(&out);

    find_all_edges(geometry);
    find_all_surface_edges(geometry);

    // spdlog::info("Mesh out:");
    // spdlog::info("  vertices num = {}", out.numberofpoints);
    // spdlog::info("  facets num = {}", out.numberoffacets);
    // spdlog::info("  trifaces num = {}", out.numberoftrifaces);
    // spdlog::info("  edges num = {}", out.numberofedges);
    // spdlog::info("  tetra num = {}", out.numberoftetrahedra);

    return geometry;
}
