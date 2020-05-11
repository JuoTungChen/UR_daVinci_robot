#ifndef RW_INVKIN_WEIGHTEDJACOBIAN_HPP
#define RW_INVKIN_WEIGHTEDJACOBIAN_HPP

#include <rw/common/Ptr.hpp>
#include <rw/invkin/IterativeIK.hpp>
#include <rw/kinematics/FKRange.hpp>

#include <vector>

namespace rw {
namespace models {
    class Device;
    class JacobianCalculator;
} // namespace models
} // namespace rw

namespace rw {
namespace invkin {

    class WeightedJacobianIKSolver : public IterativeIK
    {
    public:
        typedef rw::common::Ptr<WeightedJacobianIKSolver> Ptr;
        typedef rw::common::Ptr<const WeightedJacobianIKSolver> CPtr;

        WeightedJacobianIKSolver(rw::common::Ptr<const rw::models::Device> device,
                                 const kinematics::State& state);

        WeightedJacobianIKSolver(rw::common::Ptr<const rw::models::Device> device,
                                 const rw::kinematics::Frame* foi,
                                 const kinematics::State& state);

        std::vector<math::Q> solve(const math::Transform3D<>& baseTend,
                                   const kinematics::State& state) const;
        void setInterpolatorStep(double interpolatorStep) { _interpolationStep = interpolatorStep; }
        void setEnableInterpolation(bool enableInterpolation) { _useInterpolation = enableInterpolation; };
        bool solveLocal(const math::Transform3D<>& bTed,
                        double maxError,
                        kinematics::State& state,
                        int maxIter) const;
        void setClampToBounds(bool enableClamping) { _useJointClamping = enableClamping; };
        void setWeightVector(Eigen::VectorXd weights);
        void setCheckJointLimits(bool check) { _checkJointLimits = check; }

        virtual rw::common::Ptr<const rw::kinematics::Frame> getTCP() const;

    private:
        rw::common::Ptr<const rw::models::Device> _device;
        double _interpolationStep;
        kinematics::FKRange _fkrange;
        rw::common::Ptr<models::JacobianCalculator> _devJac;
        bool _useJointClamping, _useInterpolation, _checkJointLimits;
        Eigen::MatrixXd _wInv; // Weight matrix
    };

} // namespace invkin
} // namespace rw

#endif // end include guard
