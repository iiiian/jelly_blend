#pragma once

#include <string>

#include "body.h"

class Vertex
{
  public:
    const Body *pbody;
    size_t vertex_index;

    Vertex(const Body *pbody, size_t vertex_index) : pbody(pbody), vertex_index(vertex_index){};
};

class Face
{
  public:
    const Body *pbody;
    size_t face_index;

    Face(){};
    Face(const Body *pbody, size_t face_index) : pbody(pbody), face_index(face_index){};
};
