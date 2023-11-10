#pragma once

#include <Eigen/Core>
#include <pybind11/pybind11.h>
#include <string>

#include "utils.h"

size_t py_len(pybind11::object obj);
pybind11::object get_bl_object(std::string name);
void bl_frame_set(int frame);

class BlEvalMesh
{
  private:
    pybind11::object bl_obj_eval;

  public:
    pybind11::object bl_mesh;

    BlEvalMesh(pybind11::object bl_obj, int frame = -1);
    ~BlEvalMesh();

    std::string summary();
};

class BlVertices
{
  private:
    pybind11::object bl_vertices;

  public:
    BlVertices(pybind11::object bl_vertices);
    void copy_to(VERTICES &target_vertices, bool preserve_vert_num = false);
};

class BlTransformer
{
  private:
    Eigen::Vector3d translation;
    Eigen::Vector3d scale;
    Eigen::Matrix3d rotation;

  public:
    BlTransformer(){};
    void load_transformer(pybind11::object obj);
    Eigen::Vector3d to_global(Eigen::Ref<const Eigen::Vector3d> vec);
    void to_global_inplace(Eigen::Ref<Eigen::Vector3d> vec);
    Eigen::Vector3d to_local(Eigen::Ref<const Eigen::Vector3d> vec);
    void to_local_inplace(Eigen::Ref<Eigen::Vector3d> vec);

    std::string summary();

    template <class Archive> void serialize(Archive &ar)
    {
        ar(translation, scale, rotation);
    }
};