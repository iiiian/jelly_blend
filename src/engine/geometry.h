#pragma once

#include <Eigen/Core>
#include <string>

#include "body.h"

class Vertex
{
  public:
    Body const *pbody;
    size_t vertex_index;

    Vertex(Body const *pbody, size_t vertex_index) : pbody(pbody), vertex_index(vertex_index){};
};

class Face
{
  public:
    Body const *pbody;
    size_t face_index;

    Face(){};
    Face(Body const *pbody, size_t face_index) : pbody(pbody), face_index(face_index){};
};
