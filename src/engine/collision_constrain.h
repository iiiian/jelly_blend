#pragma once

#include <Eigen/Core>
#include <array>
#include <memory>
#include <string>

#include "collision.h"
#include "utils.h"

class CollisionConstrain
{
  public:
    virtual ~CollisionConstrain(){};
    virtual void solve(Eigen::VectorXd &x) = 0;
    virtual std::string summary() = 0;
};

using SPCollisionConstrain = std::shared_ptr<CollisionConstrain>;
using WPCollisionConstrain = std::weak_ptr<CollisionConstrain>;
using UPCollisionConstrain = std::unique_ptr<CollisionConstrain>;

// a dynamic vertex colliding on a fixed face:
//
// distance = (x - x_collision) dot face_normal >= 0
//
// defined as
// (x + c1) dot c2 >= 0
// where
// c1 = -x_collision
// c2 = face_normal
class FixedFaceCollisionConstrain : public CollisionConstrain
{
  public:
    size_t x_start;
    Eigen::Vector3d c1;
    Eigen::Vector3d c2;

    FixedFaceCollisionConstrain(const Collision &collision, const Segment &seg_vertex);
    void solve(Eigen::VectorXd &x) override;
    std::string summary() override;
};

using SPFixedFaceCollisionConstrain = std::shared_ptr<FixedFaceCollisionConstrain>;
using WPFixedFaceCollisionConstrain = std::weak_ptr<FixedFaceCollisionConstrain>;
using UPFixedFaceCollisionConstrain = std::unique_ptr<FixedFaceCollisionConstrain>;

// a fixed vertex colliding on a dynamic face
class FixedVertCollisionConstrain : public CollisionConstrain
{
  public:
    Eigen::Vector3d vert_coor;
    std::array<size_t, 3> face_x_start;
    // weight is 1/mass
    std::array<double, 3> face_weights;

    FixedVertCollisionConstrain(const Collision &collision, const Segment &seg_face);
    void solve(Eigen::VectorXd &x) override;
    std::string summary() override;
};

using SPFixedVertCollisionConstrain = std::shared_ptr<FixedVertCollisionConstrain>;
using WPFixedVertCollisionConstrain = std::weak_ptr<FixedVertCollisionConstrain>;
using UPFixedVertCollisionConstrain = std::unique_ptr<FixedVertCollisionConstrain>;

// a dynamic vertex colliding on a dynamic face:
//
// fc = face corner
// distance = (x - fc0) dot ( (fc1 - fc0) cross (fc2-fc0) ) >= 0
class DynamicCollisionConstrain : public CollisionConstrain
{
  public:
    size_t vertex_x_start;
    double vertex_weight;
    std::array<size_t, 3> face_x_start;
    // weight is 1/mass
    std::array<double, 3> face_weights;

    DynamicCollisionConstrain(const Collision &collision, const Segment &seg_vertex, const Segment &seg_face);
    void solve(Eigen::VectorXd &x) override;
    std::string summary() override;
};

using SPDynamicCollisionConstrain = std::shared_ptr<DynamicCollisionConstrain>;
using WPDynamicCollisionConstrain = std::weak_ptr<DynamicCollisionConstrain>;
using UPDynamicCollisionConstrain = std::unique_ptr<DynamicCollisionConstrain>;
