#ifndef __OPS_WBC_ROBOT_CONTROLLER_ORIENTATION_CONTROL_HPP
#define __OPS_WBC_ROBOT_CONTROLLER_ORIENTATION_CONTROL_HPP

#include <ops_wbc_robot/ops_wbc_robot.hpp>
#include "ops_wbc_robot_controller/task.hpp"

namespace ops_wbc_robot_controller
{

  class OrientationControl : public Task
  {
        public:
            explicit OrientationControl(const ops_wbc_robot::ManipulatorPtr& mnp, const std::string& target_link, double eigen_thresh = 0.001);
            virtual const std::string& getName() const override { return task::ORIENTATION_CONTROL; }
            virtual void setGoal(const Eigen::MatrixXd& Rd) override;
            virtual void setGoal(const Eigen::MatrixXd& Rd, const Eigen::MatrixXd& dRd) override;   
            virtual void updateModel() override;
            virtual void computeGeneralizedForce(Eigen::VectorXd& tau) override;
            virtual bool haveNullSpace() override { return true; }

        private:
            bool updated_;

            std::string target_link_;
            Eigen::Matrix3d Rd_;
            Eigen::Matrix3d dRd_;
            bool dRd_exist_;
            int32_t idx_;
            Eigen::MatrixXd Jw_;
            Eigen::Matrix3d lambda_inv_;
            Eigen::Matrix3d lambda_;
            Eigen::MatrixXd J_dyn_inv_;
            Eigen::MatrixXd I_;

            Eigen::Vector3d M_unit_;
            double eigen_thresh_;
  };

} // namespace ops_wbc_robot_controller

#endif // __OPS_WBC_ROBOT_CONTROLLER_ORIENTATION_CONTROL_HPP
