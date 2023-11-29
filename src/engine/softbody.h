#pragma once

#include <Eigen/Eigen>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <memory>
#include <pybind11/pybind11.h>
#include <string>
#include <vector>

#include "body.h"
#include "eigen_alias.h"
#include "eigen_serialization.h"
#include "softbody_constrain.h"

struct SoftBodySetting
{
    double density = 1;
    double gravity = 9.8;
    double youngs_modulus = 1e7;
    double poissons_ratio = 0.49;
    double damping = 0;
    double friction = 0;
    bool detect_self_collision = false;
};

class SoftBody : public Body
{
  public:
    // user settings
    double density = 10;         // kg/m^3
    double gravity = 9.8;        // m / s^2
    double youngs_modulus = 1e7; // Pa
    double poissons_ratio = 0.49;
    double damping = 0;
    double friction = 0;
    bool detect_self_collision = false;
    // load from blender data
    std::string bl_object_name;
    size_t tetra_num = 0;
    size_t surface_vertex_num = 0;
    size_t surface_edge_num = 0;

    TETRAHEDRA tetrahedra;
    VERTICES initial_vertices;
    EDGES surface_edges;

    // generated on construct
    VERTICES constant_accels;
    Eigen::VectorXd vertex_masses;
    std::vector<SPSoftBodyConstrain> sp_soft_constrains;

    SoftBody(){};
    SoftBody(pybind11::object bl_softbody);
    ~SoftBody(){};

    void load_settings(const SoftBodySetting &settings);
    void load_bl_softbody_mesh(pybind11::object bl_softbody);
    void init();
    void construct_vertex_masses();
    void construct_neonewton_constrains();
    void construct_constant_accel();

    void update_avg_predict_edge_length();
    void predict(double time_delta);

    std::string summary() const override;

    template <typename Archive> void save(Archive &ar) const
    {
        // base class Body
        ar(vertex_num, edge_num, face_num, avg_predict_edge_length);
        ar(vertices, velocity, predict_vertices, edges, faces);
        // softbody only
        ar(bl_object_name, density, gravity, youngs_modulus, poissons_ratio, damping, detect_self_collision);
        ar(tetra_num, surface_vertex_num, surface_edge_num);
        ar(tetrahedra, initial_vertices, surface_edges);
    }

    template <typename Archive> void load(Archive &ar)
    {
        // base class Body
        ar(vertex_num, edge_num, face_num, avg_predict_edge_length);
        ar(vertices, velocity, predict_vertices, edges, faces);
        // softbody only
        ar(bl_object_name, density, gravity, youngs_modulus, poissons_ratio, damping, detect_self_collision);
        ar(tetra_num, surface_vertex_num, surface_edge_num);
        ar(tetrahedra, initial_vertices, surface_edges);

        init();
    }
};

class SoftBodyMesh
{
  public:
    std::string bl_object_name;
    size_t vertex_num;
    size_t surface_vertex_num;
    std::vector<double> vertices;

    SoftBodyMesh(){};
    SoftBodyMesh(const SoftBody &soft);
};

using SPSoftBody = std::shared_ptr<SoftBody>;
using WPSoftBody = std::weak_ptr<SoftBody>;
using UPSoftBody = std::unique_ptr<SoftBody>;
