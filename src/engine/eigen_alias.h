#pragma once

#include <Eigen/Core>

using VERTICES = Eigen::Matrix<double, 3, Eigen::Dynamic>;
using EDGES = Eigen::Matrix<size_t, 2, Eigen::Dynamic>;
using FACES = Eigen::Matrix<size_t, 3, Eigen::Dynamic>;
using TETRAHEDRA = Eigen::Matrix<size_t, 4, Eigen::Dynamic>;
