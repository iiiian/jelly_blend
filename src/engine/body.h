#pragma once

#include <memory>
#include <string>

#include "eigen_alias.h"

class Body
{
  public:
    size_t vertex_num;
    size_t edge_num;
    size_t face_num;

    VERTICES vertices;
    VERTICES velocity;
    VERTICES predict_vertices;
    EDGES edges;
    FACES faces;

    virtual ~Body(){};

    virtual std::string summary() const = 0;
};

using SPBody = std::shared_ptr<Body>;
using WPBody = std::weak_ptr<Body>;
using UPBody = std::unique_ptr<Body>;