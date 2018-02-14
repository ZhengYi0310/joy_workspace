#ifndef __OPS_WBC_ROBOT_CONTROLLER_TASK_HPP
#define __OPS_WBC_ROBOT_CONTROLLER_TASK_HPP

#include <memory>
#include <Eigen/Dense>
#include <ops_wbc_robot/ops_wbc_robot.hpp>
#include <ops_wbc_robot_controller/param_base.hpp>

namespace ops_wbc_robot_controller
{
    namespace task
    {
        static const std::string UNDEFINED =                "undefined";
        static const std::string DAMPING =                  "damping";
        static const std::string FRICTION_COMPENSATION =    "friction_compensation";
        static const std::string GRAVITY_COMPENSATION  =    "gravity_compensation";
        static const std::string JOINT_CONTROL =            "joint_control";
        static const std::string JOINT_LIMIT =              "joint_limit";
        static const std::string ORIENTATION_CONTROL =      "orientation_control";
        static const std::string POSITION_CONTROL =         "position_control";
    }

    class Task 
    {
        public:
            explicit Task() = default;
            virtual ~Task() = default;
            virtual const std::string& getName() const {return task::UNDEFINED;}
            virtual void setParam(const ParamBasePtr& param) {param_ = param;}
            virtual void setGoal(const Eigen::MatrixXd& dst) {}
            virtual void setGoal(const Eigen::MatrixXd& dst1, const Eigen::MatrixXd& dst2) {}
            virtual void setGoal(const Eigen::MatrixXd& dst1, const Eigen::MatrixXd& dst2, const Eigen::MatrixXd& dst3){}
            virtual void updateModel() {}
            virtual void computeGeneralizedForce(Eigen::VectorXd& tau) {}

            virtual bool haveNullSpace() {return false;}
            virtual const Eigen::MatrixXd getNullSpace() const {return N_;}
            virtual const std::string& getTargetName() const {return mnp_->getName();}
            virtual const Eigen::VectorXd& getTorque() const {return tau_;}

        protected:
            ops_wbc_robot::ManipulatorPtr mnp_;
            Eigen::VectorXd tau_;
            Eigen::MatrixXd N_;
            ParamBasePtr param_;
    };

    using TaskPtr = std::shared_ptr<Task>;
}

#endif
