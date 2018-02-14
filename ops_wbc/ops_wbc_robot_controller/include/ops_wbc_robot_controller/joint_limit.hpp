#ifndef __OPS_WBC_ROBOT_CONTROLLER_JOINT_LIMIT_HPP
#define __OPS_WBC_ROBOT_CONTROLLER_JOINT_LIMIT_HPP

#include "ops_wbc_robot_controller/task.hpp"

namespace ops_wbc_robot_controller
{
    class JointLimit : public Task 
    {
        public:
            explicit JointLimit(const ops_wbc_robot::ManipulatorPtr& mnp, double threshold);
            virtual const std::string& getName() const override {return task::JOINT_LIMIT;}
            virtual void computeGeneralizedForce(Eigen::VectorXd& tau) override;
            virtual bool haveNullSpace() override {return true;}

        private:
            bool moveAwayFromMax(double q, double max);
            bool moveAwayFromMin(double q, double min);

            Eigen::VectorXd q_max_;
            Eigen::VectorXd q_min_;

            std::vector<bool> lock_;
            double threshold_;
    };
}

#endif // __OPS_WBC_ROBOT_CONTROLLER_JOINT_LIMIT_HPP
