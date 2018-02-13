#ifndef __OPS_WBC_ROBOT_CONTROLLER_EFFECTIVE_MASS_MATRIX3D_HPP
#define __OPS_WBC_ROBOT_CONTROLLER_EFFECTIVE_MASS_MATRIX3D_HPP

#include <Eigen/Dense>
namespace ops_wbc_robot_controller
{
    class EffectiveMassMatrix3d
    {
        public:
            static void compute(const Eigen::Matrix3d& lambda_inv, Eigen::Matrix3d& lambda, double thresh=0.0);
    };
}

#endif // __OPS_WBC_ROBOT_CONTROLLER_EFFECTIVE_MASS_MATRIX3D_HPP
