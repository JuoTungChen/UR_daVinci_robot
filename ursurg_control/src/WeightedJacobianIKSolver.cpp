/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "WeightedJacobianIKSolver.hpp"

#include <rw/math/Jacobian.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/VelocityScrew6D.hpp>

#include <rw/models/Device.hpp>
#include <rw/models/Models.hpp>

#include <rw/kinematics/Frame.hpp>

#include <rw/trajectory/LinearInterpolator.hpp>

using namespace boost;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::invkin;
using namespace rw::trajectory;

WeightedJacobianIKSolver::WeightedJacobianIKSolver(Device::CPtr device, const Frame* foi, const State& state)
    : _device(device)
    , _interpolationStep(0.21)
    , _fkrange(device->getBase(), foi, state)
    , _devJac(device->baseJCframe(foi, state))
    , _useJointClamping(false)
    , _useInterpolation(false)
    , _checkJointLimits(false)
{
    setMaxIterations(15);
}

WeightedJacobianIKSolver::WeightedJacobianIKSolver(Device::CPtr device, const State& state)
    : _device(device)
    , _interpolationStep(0.21)
    , _fkrange(device->getBase(), device->getEnd(), state)
    , _devJac(device->baseJCend(state))
    , _useJointClamping(false)
    , _useInterpolation(false)
    , _checkJointLimits(false)
{
    setMaxIterations(15);
}

std::vector<Q> WeightedJacobianIKSolver::solve(const Transform3D<>& bTed,
                                               const State& initial_state) const
{
    int maxIterations = getMaxIterations();
    double maxError = getMaxError();
    State state = initial_state;

    // if the distance between current and end configuration is
    // too large then split it up in smaller steps
    auto bTeInit = _fkrange.get(state);

    if (_useInterpolation) {
        LinearInterpolator<Transform3D<>> interpolator(bTeInit, bTed, _interpolationStep);

        for (double t = _interpolationStep; t < interpolator.duration(); t += _interpolationStep) {
            auto bTed_via = interpolator.x(t);
            solveLocal(bTed_via, maxError * 1000, state, 5);
        }
    }

    // now we perform yet another newton search with higher precision to determine
    // the end result
    if (solveLocal(bTed, maxError, state, maxIterations)) {
        std::vector<Q> result;
        auto q = _device->getQ(state);

        if (!_checkJointLimits || Models::inBounds(q, *_device))
            result.push_back(q);

        return result;
    }

    return std::vector<Q>();
}

bool WeightedJacobianIKSolver::solveLocal(const Transform3D<>& bTed,
                                          double maxError,
                                          State& state,
                                          int maxIter) const
{
    Q q = _device->getQ(state);
    Device::QBox bounds = _device->getBounds();

    for (int cnt = 0; cnt < maxIter; ++cnt) {
        const Transform3D<>& bTe = _fkrange.get(state);
        const Transform3D<>& eTed = inverse(bTe) * bTed;

        const EAA<> e_eOed(eTed(2, 1), eTed(0, 2), eTed(1, 0));
        const Vector3D<>& e_eVed = eTed.P();
        const VelocityScrew6D<> e_eXed(e_eVed, e_eOed);
        const VelocityScrew6D<>& b_eXed = bTe.R() * e_eXed;

        if (normInf(b_eXed) <= maxError)
            return true;

        Eigen::VectorXd dS = b_eXed.e();
        Eigen::MatrixXd J = _devJac->get(state).e();
        auto JT = J.transpose();

        // Equation 9 from https://ieeexplore.ieee.org/document/370511
        Eigen::MatrixXd Jwp = _wInv * JT * (J * _wInv * JT).inverse();
        Q dq(Jwp * dS);
        double dq_len = dq.normInf();

        if (dq_len > 0.8)
            dq *= 0.8 / dq_len;

        q += dq;

        if (_useJointClamping)
            q = Math::clampQ(q, bounds.first, bounds.second);

        _device->setQ(q, state);
    }

    return false;
}

void WeightedJacobianIKSolver::setWeightVector(Eigen::VectorXd weights)
{
    if (weights.size() != _device->getDOF())
        throw std::runtime_error("Weight vector must have same length as device DOF!");

    _wInv = weights.asDiagonal().inverse();
}

rw::kinematics::Frame::CPtr WeightedJacobianIKSolver::getTCP() const
{
    return _fkrange.getEnd();
}
