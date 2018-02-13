#ifndef __OPS_WBC_ROBOT_GRAVITY_COMPENSATION_HPP
#define __OPS_WBC_ROBOT_GRAVITY_COMPENSATION_HPP

#include "ops_wbc_robot_controller/task.hpp"

namespace ops_wbc_robot_controller
{
    class GravityCompensation : public Task 
    {
        public:
            explicit GravityCompensation(const ops_wbc_robot::RobotPtr& robot);
            virtual const std::string& getName() const override {return task::GRAVITY_COMPENSATION;}
            virtual void computeGeneralizedForce(Eigen::VectorXd& tau) override;
            virtual const std::string& getTargetName() const override {return robot_->getName();}

        private:
            ops_wbc_robot::RobotPtr robot_;
            std::vector<std::string> mnp_name_;
    };
}

#endif // __OPS_WBC_ROBOT_GRAVITY_COMPENSATION_HPP
