#pragma once

#include <Eigen/Core>
#include <cereal/types/optional.hpp>
#include <optional>
#include <string>
#include <vector>

#include "body.h"
#include "fixedbody.h"
#include "geometry.h"
#include "softbody.h"

class Collision
{
  public:
    const Vertex vertex;
    const Face face;
    Eigen::Vector3d face_normal;
    Eigen::Vector3d cpoint; // collision point

    Collision(const Vertex vertex, const Face face, Eigen::Vector3d face_normal, Eigen::Vector3d collision_point);
    std::string summary();
};

class VertexTrajectory
{
  public:
    Vertex vertex;
    Eigen::Vector3d coor;
    Eigen::Vector3d predict_coor;

    VertexTrajectory(const Vertex &vertex, const Eigen::Ref<const Eigen::Vector3d> coor,
                     const Eigen::Ref<const Eigen::Vector3d> predict_coor)
        : vertex(vertex), coor(coor), predict_coor(predict_coor){};
};

struct SpatialMapElement
{
    double time = -1;
    bool is_vert_uni = true;
    Body const *pvert_first = nullptr;
    std::vector<VertexTrajectory> vert_trajectories;
};

struct FaceRunTimeInfo
{
    const Body *pbody;
    size_t face_index;
    Eigen::Vector<size_t, 3> vert_indexes;
    Eigen::Matrix3d coor;
    Eigen::Matrix3d predict_coor;
};

class CollisionDetector
{
  private:
    std::vector<SpatialMapElement> spatial_map;
    std::vector<Collision> collisions;
    size_t vert_traj_num = 0;
    size_t spatial_map_size = 1;
    double spatial_cell_size = 1;
    double passive_collision_distance = 0;

    FaceRunTimeInfo load_face_runtime_info(const Body &body, size_t face_index);
    void spatial_map_housekeeping();
    void calcualte_spatial_cell_size(const std::vector<const SoftBody *> &p_softbodies,
                                     const std::vector<const FixedBody *> &p_fixedbodies, double time_delta);

    void spatial_map_insert(size_t hash, double time, const VertexTrajectory &vert_traj);
    size_t point_to_hash(const Eigen::Ref<const Eigen::Vector3d> &point);
    void cal_3x3_minmax(const Eigen::Matrix3d &coor, Eigen::Vector3d &min, Eigen::Vector3d &max);
    std::vector<size_t> minmax_to_hash(const Eigen::Vector3d &min, const Eigen::Vector3d &max);
    void hash_soft_to_spatial_map(const SoftBody &soft, double time);

    Eigen::Vector3d cal_face_bary(const Eigen::Vector3d &vert, const Eigen::Matrix3d &face);
    Eigen::Vector3d cal_face_bary(const Eigen::Vector3d &vert, const Eigen::Matrix3d &face, Eigen::Vector3d &normal,
                                  Eigen::Vector3d &vert_projection);
    bool is_traj_penetration(const Eigen::Vector3d &predict_bary, const Eigen::Vector3d &bary);
    void detect_face_soft_vert_colli(const VertexTrajectory &vert_traj, const FaceRunTimeInfo &face_info);
    void detect_face_collisions(const Body &body, size_t face_index, double time);
    void detect_body_collisions(const Body &body, double time);

  public:
    unsigned int spatial_map_size_multiplier = 50;
    // in MB, if the size of the vertex trajectories in spatial map excedes this limits, clean
    // the map
    size_t spatial_map_mem_limit = 1024;
    std::optional<double> manual_spatial_cell_size;
    std::optional<double> manual_passive_collision_distance;
    // the difference in dimension of a face between 1 time step should not exceed this limit
    unsigned int blow_up_limit = 1e2;

    void reset_spatial_map();
    void clear_spatial_map();
    std::vector<Collision> detect_collisions(const std::vector<const SoftBody *> &p_softbodies,
                                             const std::vector<const FixedBody *> &p_fixedbodies, double time,
                                             double time_delta);
    std::string summary();

    template <typename Archive> void serialize(Archive &ar)
    {
        ar(spatial_map_size_multiplier, spatial_map_mem_limit, manual_spatial_cell_size, spatial_cell_size,
           manual_passive_collision_distance, passive_collision_distance, blow_up_limit);
    }
};