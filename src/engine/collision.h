#pragma once

#include <Eigen/Core>
#include <string>

#include "body.h"
#include "geometry.h"

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