#pragma once

#include <pronto_quadruped_commons/feet_jacobians.h>
#include <anymal_robcogen/jacobians.h>

namespace pronto {
namespace anymal {

class FeetJacobians : public pronto::quadruped::FeetJacobians {
public:
    typedef pronto::quadruped::FootJac FootJac;
    typedef pronto::quadruped::JointState JointState;
    typedef pronto::quadruped::LegID LegID;

    inline FeetJacobians() {}

    virtual ~FeetJacobians() {}

    FootJac getFootJacobian(const JointState& q, const LegID& leg);
    FootJac getFootJacobianLF(const JointState& q);
    FootJac getFootJacobianRF(const JointState& q);
    FootJac getFootJacobianLH(const JointState& q);
    FootJac getFootJacobianRH(const JointState& q);

    FootJac getFootJacobian(const JointState &q, const LegID &leg,
                            const double& foot_x, const double& foot_y);

private:
    pronto::anymal::Jacobians jacs_;
};

}
}

