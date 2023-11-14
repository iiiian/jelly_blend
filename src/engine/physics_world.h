#pragma once

#include <cereal/access.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>
#include <map>
#include <optional>
#include <pybind11/pybind11.h>
#include <vector>

#include "body.h"
#include "collision.h"
#include "collision_constrain.h"
#include "fixedbody.h"
#include "geometry.h"
#include "softbody.h"
#include "utils.h"

struct SpatialMapElement
{
    double time = -1;
    bool is_face_uni = true;
    const Body *pbody_first = nullptr;
    std::vector<Face> faces;
};

struct PhysicsWorldSetting
{
    int solver_substep_num = 1;
    int frame_substep_num = 50;
    int frame_rate = 24;
    double passive_collision_distance = 0.1;
    size_t spatial_map_mem_threshold = 100;
    int spatial_map_size_multiplier = 50;
};

class PhysicsWorld
{
  private:
    // set by user
    // each substep only invole solving constrains
    int solver_substep_num = 1;
    // each substep has damping, collision handling, ...
    int frame_substep_num = 50;
    int frame_rate = 24;
    // collision below this distance will be detected
    double passive_collision_distance = 1e-2;
    // in MB, if the size of the faces in spatial map excedes this limits, clean
    // the map
    size_t spatial_map_mem_threshold = 300;
    int spatial_map_size_multiplier = 50;

    // generated automatically, persistant across simulations
    size_t spatial_map_size = 0;
    size_t spatial_map_mem_occupation = 0;
    size_t position_num = 0;
    std::vector<SPFixedBody> sp_fixedbodies;
    std::vector<SPSoftBody> sp_softbodies;
    std::map<Body *, Segment> softbody_positions;

    // generated automatically during every simulation
    bool test_mode = false; // if true, no mesh key frame insertions
    int current_frame = 0;
    int current_frame_substep = 0;
    double time_delta = 0; // for a frame substep
    double time = 0;
    double spatial_cell_size = 0;

    std::vector<SpatialMapElement> spatial_map;
    std::vector<Collision> collisions;
    std::vector<SPCollisionConstrain> sp_colli_constrains;
    Eigen::VectorXd simulation_vars;

    void clean_spatial_map();
    void update_spatial_map_cellsize();

    size_t point_to_hash(const Eigen::Ref<const Eigen::Vector3d> &point);
    std::vector<size_t> minmax_to_hash(const Eigen::Vector3d &min, const Eigen::Vector3d &max);
    void predict_face_to_spatial_map(Body *pbody, size_t face_index);
    Eigen::Vector3d cal_face_bary(const Eigen::Vector3d &vert, const Eigen::Matrix3d &face);
    Eigen::Vector3d cal_face_bary(const Eigen::Vector3d &vert, const Eigen::Matrix3d &face, Eigen::Vector3d &normal,
                                  Eigen::Vector3d &vert_projection);
    std::optional<Collision> detect_soft_vertex_collision(SoftBody *psoft, size_t vertex_index, size_t hash);
    void detect_collisions();
    void generate_colli_constrains();

    void predict();
    void prepare_simulation_vars();
    void solve_predict();

    void update_body_predict();
    void update_body_vertices();
    void reindex_softbody_positions();

    void update();

    friend class cereal::access;

    template <typename Archive> void save(Archive &ar) const
    {
        ar(solver_substep_num, frame_substep_num, frame_rate, passive_collision_distance);
        ar(spatial_map_size, position_num);
        ar(sp_fixedbodies, sp_softbodies);
    }

    template <typename Archive> void load(Archive &ar)
    {
        ar(solver_substep_num, frame_substep_num, frame_rate, passive_collision_distance);
        ar(spatial_map_size, position_num);
        ar(sp_fixedbodies, sp_softbodies);
        reindex_softbody_positions();
    }

  public:
    PhysicsWorld(){};
    ~PhysicsWorld(){};
    void add_softbody(SPSoftBody sp_soft);
    void add_softbody_bl(pybind11::object bl_softbody, SoftBodySetting setting);
    void add_fixedbody(SPFixedBody sp_fixed);
    void add_fixedbody_bl(pybind11::object bl_fixedbody);
    void prepare_simulation(int frame_start, bool test_mode);
    void next_frame();
    void simulate(int frame_start, int frame_end, bool test_mode = false);
    void load_setting(const PhysicsWorldSetting &setting);
    void dump_to_file(std::string file_path);
    void load_from_file(std::string file_path);
    std::string summary();
    void test_func();
};