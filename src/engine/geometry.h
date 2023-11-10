#pragma once

#include <Eigen/Core>
#include <string>

#include "body.h"

struct Vertex
{
    Body *pbody;
    size_t vertex_index;
};

class Face
{
  public:
    Body *pbody;
    size_t face_index;

    Face(){};
    Face(Body *pbody, size_t face_index) : pbody(pbody), face_index(face_index){};
};
