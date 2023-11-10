#pragma once

#include <Eigen/Core>
#include <array>
#include <cereal/types/array.hpp>
#include <memory>
#include <string>

class SoftBodyConstrain
{
  public:
    virtual ~SoftBodyConstrain(){};
    virtual void reset() = 0;
    virtual void solve(Eigen::Ref<Eigen::VectorXd> x, double time_delta) = 0;
    virtual std::string summary() = 0;
};

using SPSoftBodyConstrain = std::shared_ptr<SoftBodyConstrain>;
using WPSoftBodyConstrain = std::weak_ptr<SoftBodyConstrain>;
using UPSoftBodyConstrain = std::unique_ptr<SoftBodyConstrain>;

class NeoNewtonConstrain : public SoftBodyConstrain
{
  private:
    double dev_compliance;
    double hy_compliance;
    double hy_offset;
    Eigen::Matrix<double, 3, 3> ir;
    Eigen::Matrix<double, 3, 4> ic; // initial corner coordinates
    Eigen::Vector4d corner_weights;
    std::array<size_t, 4> corner_positions;
    double dev_lambda = 0;
    double hy_lambda = 0;

  public:
    NeoNewtonConstrain(double lame1, double lame2, const Eigen::Matrix<double, 3, 4> &init_corners,
                       const Eigen::Vector4d &corner_weights, const std::array<size_t, 4> &corner_positions);
    void reset() override;
    void solve(Eigen::Ref<Eigen::VectorXd> x, double time_delta) override;
    std::string summary() override;
};

using SPNeoNewtonConstrain = std::shared_ptr<NeoNewtonConstrain>;
using WPNeoNewtonConstrain = std::weak_ptr<NeoNewtonConstrain>;
using UPNeoNewtonConstrain = std::unique_ptr<NeoNewtonConstrain>;