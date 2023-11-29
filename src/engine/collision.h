#pragma once

#include <Eigen/Core>
#include <optional>
#include <string>
#include <vector>

#include "body.h"
#include "geometry.h"
#include "softbody.h"

class Collision
{
  public:
    Vertex vertex;
    Face face;
    Eigen::Vector3d face_normal;
    Eigen::Vector3d cpoint; // collision point

    Collision(Vertex &vertex, Face &face, Eigen::Vector3d &face_normal, Eigen::Vector3d &collision_point);
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

class CollisionDetector
{
  private:
    std::vector<SpatialMapElement> spatial_map;
    size_t vert_traj_num = 0;

    size_t point_to_hash(const Eigen::Ref<const Eigen::Vector3d> &point);
    std::vector<size_t> minmax_to_hash(const Eigen::Vector3d &min, const Eigen::Vector3d &max);
    std::vector<size_t> face_coor_to_hashes(const Eigen::Matrix3d &face_coor);
    Eigen::Vector3d cal_face_bary(const Eigen::Vector3d &vert, const Eigen::Matrix3d &face);
    Eigen::Vector3d cal_face_bary(const Eigen::Vector3d &vert, const Eigen::Matrix3d &face, Eigen::Vector3d &normal,
                                  Eigen::Vector3d &vert_projection);
    std::vector<Collision> detect_face_collisions(const Body &body, size_t face_index, double time);

  public:
    // in MB, if the size of the vertex trajectories in spatial map excedes this limits, clean
    // the map
    size_t spatial_map_mem_limit = 300;
    size_t spatial_map_size = 1;
    double spatial_cell_size = 1;
    // if false, passive_collision_distance = spatial_cell_size / 10
    bool manual_passive_collision_distance = false;
    double passive_collision_distance = 0;
    // the upper limit of the number of spatial cells a face can span
    unsigned int blow_up_limit = 10000;

    void reset_spatial_map();
    void spatial_map_housekeeping();
    void hash_soft_to_spatial_map(const SoftBody &soft, double time);
    void detect_body_collisions(const Body &body, double time, std::vector<Collision> &collisons);
    std::string summary();

    template <typename Archive> void serialize(Archive &ar)
    {
        ar(spatial_map_mem_limit, spatial_map_size, spatial_cell_size, passive_collision_distance, blow_up_limit);
    }
};