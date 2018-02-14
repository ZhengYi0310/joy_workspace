#ifndef __OPS_WBC_ROBOT_CONTROLLER_FRICTION_COMPENSATION
#define __OPS_WBC_ROBOT_CONTROLLER_FRICTION_COMPENSATION

#include "ops_wbc_robot_controller/task.hpp"

namespace ops_wbc_robot_controller
{
    class FrictionCompensation : public Task 
    {
        public:
            explicit FrictionCompensation(const ops_wbc_robot::RobotPtr& robot);
            virtual const std::string& getName() const override {return task::FRICTION_COMPENSATION;}
            virtual void computeGeneralizedForce(Eigen::VectorXd& tau) override;
            virtual const std::string& getTargetName() const override {return robot_->getName();}

        private:
            ops_wbc_robot::RobotPtr robot_;
            Eigen::MatrixXd b_;
            std::vector<std::string> mnp_name_;
    };
}

#endif // __OPS_WBC_ROBOT_CONTROLLER_FRICTION_COMPENSATION
