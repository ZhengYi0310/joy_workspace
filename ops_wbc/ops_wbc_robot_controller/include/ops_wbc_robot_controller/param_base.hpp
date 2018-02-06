#ifndef __OPS_WBC_ROBOT_CONTROLLER_PARAM_BASE_HPP
#define __OPS_WBC_ROBOT_CONTROLLER_PARAM_BASE_HPP

#include <memory>
#include <Eigen/Dense>

namespace ops_wbc_robot_controller
{
    class ParamBase
    {
        public:
            explicit ParamBase() = default;
            virtual ~ParamBase() = default;

            virtual const Eigen::MatrixXd& getKpJoint() = 0;
            virtual const Eigen::MatrixXd& getKvJoint() = 0;
            virtual const Eigen::MatrixXd& getKpTask() = 0;
            virtual const Eigen::MatrixXd& getKiTask() = 0;
            virtual const Eigen::MatrixXd& getKvTask() = 0;
            virtual const Eigen::MatrixXd& getKvDamp() = 0;
            virtual const Eigen::MatrixXd& getKpLimit() = 0;
            virtual const Eigen::MatrixXd& getKvLimit() = 0;
            virtual const Eigen::Vector3d& getIClippingTaskPos() = 0;
            virtual const Eigen::Vector3d& getIClippingTaskOri() = 0;
            virtual double getJointErrorMax() = 0;
            virtual double getPosErrorMax() = 0;
            virtual double getOriErrorMax() = 0;
            virtual double getDqMax() = 0;
            virtual double getVxMax() = 0;
            virtual double getKpWheel() = 0;
            virtual double getKvWheel() = 0;
            virtual const Eigen::Vector3d& getG() = 0;
            virtual const Eigen::MatrixXd& getB() = 0;
    };
    using ParamBasePtr = std::shared_ptr<ParamBase>;
}

#endif // __OPS_WBC_ROBOT_CONTROLLER_PARAM_BASE_HPP
