#ifndef __OPS_WBC_ROTBO_CONTROLLER_POSITION_CONTROLLER_HPP
#define __OPS_WBC_ROTBO_CONTROLLER_POSITION_CONTROLLER_HPP

#include <ops_wbc_robot/ops_wbc_robot.hpp>
#include "ops_wbc_robot_controller/task.hpp"

namespace ops_wbc_robot_controller
{
    class PositionControl : public Task 
    {
        public:
            explicit PositionControl(const ops_wbc_robot::ManipulatorPtr& mnp, const std::string& target_link, double eigen_thresh = 0.001);
            virtual const std::string& getName() const override { return task::POSITION_CONTROL; }
            virtual void setGoal(const Eigen::MatrixXd& xd) override;
            virtual void setGoal(const Eigen::MatrixXd& xd, const Eigen::MatrixXd& dxd) override;
            virtual void updateModel() override;
            virtual void computeGeneralizedForce(Eigen::VectorXd& tau) override;
            virtual bool haveNullSpace() override { return true; }

        private:
            bool updated_;
            
            std::string target_link_;
            Eigen::Vector3d xd_;
            Eigen::Vector3d dxd_;
            bool dxd_exist_;

            int32_t idx_;
            Eigen::MatrixXd Jv_;
            Eigen::Matrix3d lambda_inv_;
            Eigen::Matrix3d lambda_;
            Eigen::MatrixXd J_dyn_inv_;
            Eigen::MatrixXd I_;

            Eigen::Vector3d F_unit_;
            double eigen_thresh_;

            Eigen::Vector3d error_sum_;
            double dt_;
    };
}
#endif


