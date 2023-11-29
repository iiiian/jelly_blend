#pragma once

#include <cereal/types/string.hpp>
#include <memory>
#include <pybind11/pybind11.h>
#include <string>

#include "body.h"
#include "eigen_serialization.h"

class FixedBody : public Body
{
  public:
    // load from bl_object/file
    std::string bl_object_name;

    // generate automatically
    double time;
    VERTICES prev_frame_vertices;
    VERTICES next_frame_vertices;

    FixedBody(){};
    FixedBody(pybind11::object bl_fixedbody);
    ~FixedBody(){};

    void update_avg_predict_edge_length();
    void update_frame_vert(int next_frame);
    // when progress=0, fixedbody at prev_frame
    // when progress=1, fixedbody at next_frame
    void predict(double progress, bool test_mode = false);
    std::string summary() const override;

    template <typename Archive> void serialize(Archive &ar)
    {
        // base class Body
        ar(vertex_num, edge_num, face_num, avg_predict_edge_length);
        ar(vertices, predict_vertices, edges, faces);

        ar(bl_object_name);
    }
};

using SPFixedBody = std::shared_ptr<FixedBody>;
using WPFixedBody = std::weak_ptr<FixedBody>;
using UPFixedBody = std::unique_ptr<FixedBody>;
