#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <sstream>

#include "softbody_constrain.h"

NeoNewtonConstrain::NeoNewtonConstrain(double lame1, double lame2, const Eigen::Matrix<double, 3, 4> &init_corners,
                                       const Eigen::Vector4d &corner_weights,
                                       const std::array<size_t, 4> &corner_positions)
    : ic(init_corners), corner_weights(corner_weights), corner_positions(corner_positions)
{

    Eigen::Matrix3d r;
    for (size_t i = 0; i < 3; ++i)
    {
        r.col(i) = ic.col(i) - ic.col(3);
    }
    double volume = -r.determinant() / 6;
    if (volume < 1e-7)
    {
        throw "vol too small";
    }

    ir = r.inverse();

    assert((volume > 0));
    assert((lame1 > 0));
    assert((lame2 > 0));
    dev_compliance = 1 / lame2 / volume;
    hy_compliance = 1 / lame1 / volume;
    hy_offset = 1 + lame2 / lame1;
}

void NeoNewtonConstrain::reset()
{
    dev_lambda = 0;
    hy_lambda = 0;
}

void NeoNewtonConstrain::solve(Eigen::Ref<Eigen::VectorXd> x, double time_delta)
{
    Eigen::Matrix<double, 3, 4> c; // current coordinate of the corners
    for (size_t i = 0; i < 4; ++i)
    {
        c.col(i) = x(Eigen::seqN(corner_positions[i], 3));
    }
    Eigen::Matrix3d r;
    for (size_t i = 0; i < 3; ++i)
    {
        r.col(i) = c.col(i) - c.col(3);
    }

    Eigen::Matrix<double, 3, 4> dc; // delta corner coordinates
    Eigen::Matrix3d F = r * ir;

    // deviatoric constrain
    double dev_alpha = dev_compliance / time_delta / time_delta;
    double dev_cons = std::pow(F.norm(), 2) - 3;

    Eigen::Matrix3d dev_r_grad = 2 * F * ir.transpose();
    Eigen::Matrix<double, 3, 4> dev_grad;
    dev_grad.col(0) = dev_r_grad.col(0);
    dev_grad.col(1) = dev_r_grad.col(1);
    dev_grad.col(2) = dev_r_grad.col(2);
    dev_grad.col(3) = -dev_r_grad.rowwise().sum();

    Eigen::Vector4d dev_grad2 = (dev_grad.array() * dev_grad.array()).colwise().sum();
    double w_dev_grad2 = dev_grad2.dot(corner_weights);
    double d_dev_lambda = (-dev_cons - dev_alpha * dev_lambda) / (w_dev_grad2 + dev_alpha);
    dev_lambda += d_dev_lambda;
    for (size_t i = 0; i < 4; ++i)
    {
        dc.col(i) = corner_weights[i] * d_dev_lambda * dev_grad.col(i);
    }

    // update F for hydrostatic constrain
    for (size_t i = 0; i < 3; ++i)
    {
        r.col(i) += dc.col(i) - dc.col(3);
    }
    F = r * ir;

    // hydrostatic constrain
    hy_offset = 1;
    double hy_alpha = hy_compliance / time_delta / time_delta;
    double hy_cons = F.determinant() - hy_offset;

    auto cal_cofactor_element = [](Eigen::Matrix3d &m, size_t i, size_t j) {
        size_t i1 = (i + 1) % 3;
        size_t i2 = (i + 2) % 3;
        size_t j1 = (j + 1) % 3;
        size_t j2 = (j + 2) % 3;

        return m(i1, j1) * m(i2, j2) - m(i1, j2) * m(i2, j1);
    };

    Eigen::Matrix3d cF; // cofactor of F
    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            cF(i, j) = cal_cofactor_element(F, i, j);
        }
    }

    Eigen::Matrix3d hy_r_grad = ir * cF.transpose(); // ir * adjugate(F)
    Eigen::Matrix<double, 3, 4> hy_grad;
    hy_grad.col(0) = hy_r_grad.row(0);
    hy_grad.col(1) = hy_r_grad.row(1);
    hy_grad.col(2) = hy_r_grad.row(2);
    hy_grad.col(3) = -hy_r_grad.colwise().sum();

    Eigen::Vector4d hy_grad2 = (hy_grad.array() * hy_grad.array()).colwise().sum();
    double w_hy_grad2 = hy_grad2.dot(corner_weights);
    double d_hy_lambda = (-hy_cons - hy_alpha * hy_lambda) / (w_hy_grad2 + hy_alpha);
    hy_lambda += d_hy_lambda;
    for (size_t i = 0; i < 4; ++i)
    {
        dc.col(i) += corner_weights[i] * d_hy_lambda * hy_grad.col(i);
    }

    for (size_t i = 0; i < 4; ++i)
    {
        x(Eigen::seqN(corner_positions[i], 3)) += dc.col(i);
    }
}

std::string NeoNewtonConstrain::summary()
{
    std::stringstream ss;

    ss << "-----------------NeoNewtonConstrain "
          "summary-----------------\n";
    ss << "deviatoric compliance = " << dev_compliance << "\n";
    ss << "hydrostatic compliance = " << hy_compliance << "\n";
    ss << "hydrostatic offset = " << hy_offset << "\n";
    ss << "inverse r = \n";
    ss << ir << "\n";
    ss << "initial corners coordinates = \n";
    ss << ic << "\n";
    ss << "corner weights = " << corner_weights.transpose() << "\n";
    ss << "corner positions = " << corner_positions[0] << " " << corner_positions[1] << " " << corner_positions[2]
       << " " << corner_positions[3] << "\n";
    ss << "deviatoric lambda = " << dev_lambda << "\n";
    ss << "hydrostatic lambda = " << hy_lambda << "\n";

    return ss.str();
}
