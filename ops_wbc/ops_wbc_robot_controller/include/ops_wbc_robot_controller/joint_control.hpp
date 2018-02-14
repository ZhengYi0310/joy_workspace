#ifndef __OPS_WBC_ROBOT_CONTROLLER_JOINT_CONTROL_HPP
#define __OPS_WBC_ROBOT_CONTROLLER_JOINT_CONTROL_HPP

#include <ops_wbc_robot/ops_wbc_robot.hpp>
#include "ops_wbc_robot_controller/task.hpp"

namespace ops_wbc_robot_controller
{
    class JointControl : public Task
    {
        public:
            explicit JointControl(const ops_wbc_robot::ManipulatorPtr& mnp);
            virtual const std::string& getName() const override {return task::JOINT_CONTROL; }
            virtual void setGoal(const Eigen::MatrixXd& qd) override;
            virtual void setGoal(const Eigen::MatrixXd& qd, const Eigen::MatrixXd& dqd) override;
            virtual void computeGeneralizedForce(Eigen::VectorXd& tau) override;

        private:
            Eigen::VectorXd qd_;
            Eigen::VectorXd dqd_;
            bool dqd_exist_;
    };
}

#endif 
