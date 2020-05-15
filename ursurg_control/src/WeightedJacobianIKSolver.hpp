#ifndef RW_INVKIN_WEIGHTEDJACOBIAN_HPP
#define RW_INVKIN_WEIGHTEDJACOBIAN_HPP

#include <rw/invkin/IterativeIK.hpp>
#include <rw/kinematics/FKRange.hpp>

namespace rw {
namespace models {

class Device;
class JacobianCalculator;

} // namespace models
} // namespace rw

class WeightedJacobianIKSolver : public rw::invkin::IterativeIK
{
public:
    typedef rw::common::Ptr<WeightedJacobianIKSolver> Ptr;
    typedef rw::common::Ptr<const WeightedJacobianIKSolver> CPtr;

    WeightedJacobianIKSolver(rw::common::Ptr<const rw::models::Device> device,
                             const rw::kinematics::State& state);
    WeightedJacobianIKSolver(rw::common::Ptr<const rw::models::Device> device,
                             const rw::kinematics::Frame* foi,
                             const rw::kinematics::State& state);

    std::vector<rw::math::Q> solve(const rw::math::Transform3D<>& baseTend,
                                   const rw::kinematics::State& state) const;
    void setInterpolatorStep(double interpolatorStep);
    void setEnableInterpolation(bool enableInterpolation);
    bool solveLocal(const rw::math::Transform3D<>& bTed,
                    double maxError,
                    rw::kinematics::State& state,
                    int maxIter) const;
    void setClampToBounds(bool enableClamping);
    void setWeightVector(Eigen::VectorXd weights);
    Eigen::VectorXd getWeightVector() const;
    void setCheckJointLimits(bool check);

    virtual rw::common::Ptr<const rw::kinematics::Frame> getTCP() const;

private:
    rw::common::Ptr<const rw::models::Device> _device;
    double _interpolationStep;
    rw::kinematics::FKRange _fkrange;
    rw::common::Ptr<rw::models::JacobianCalculator> _devJac;
    bool _useJointClamping;
    bool _useInterpolation;
    bool _checkJointLimits;
    Eigen::MatrixXd _w; // Weight matrix
    Eigen::MatrixXd _wInv;
};

#endif // end include guard
