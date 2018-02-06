#ifndef __OPS_WBC_ROBOT_CONTROLLER_PARAM_HPP
#define __OPS_WBC_ROBOT_CONTROLLER_PARAM_HPP

#include <memory>
#include <mutex>
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <ops_wbc_robot/robot/robot.hpp>
#include <utils/scoped_lock.hpp>
#include "ops_wbc_robot_controller/param_base.hpp"
#include "ops_wbc_robot_controller/ParamConfig.h"

namespace ops_wbc_robot_controller
{
    class Param : public ParamBase
    {
        public:
            explicit Param(const ops_wbc_robot::RobotPtr& robot);

            virtual const Eigen::MatrixXd& getKpJoint() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return Kp_joint_;
            }

            virtual const Eigen::MatrixXd& getKvJoint() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return Kv_joint_;
            }

            virtual const Eigen::MatrixXd& getKpTask() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return Kp_task_;
            }

            virtual const Eigen::MatrixXd& getKiTask() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return Ki_task_;
            }

            virtual const Eigen::MatrixXd& getKvTask() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return Kv_task_;
            }

            virtual const Eigen::Vector3d& getIClippingTaskPos() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return i_clipping_task_pos_;
            }

            virtual const Eigen::Vector3d& getIClippingTaskOri() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return i_clipping_task_ori_;
            }

            virtual const Eigen::MatrixXd& getKvDamp() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return Kv_damp_;
            }

            virtual const Eigen::MatrixXd& getKpLimit() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return Kp_limit_;
            }

            virtual const Eigen::MatrixXd& getKvLimit() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return Kv_limit_;
            }

            virtual double getJointErrorMax() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return joint_error_max_;
            }

            virtual double getPosErrorMax() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return pos_error_max_;
            }

            virtual double getOriErrorMax() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return ori_error_max_;
            }

            virtual double getDqMax() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return dq_max_;
            }

            virtual double getVxMax() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return vx_max_;
            }

            virtual const Eigen::Vector3d& getG() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return g_;
            }

            virtual const Eigen::MatrixXd& getB() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return b_;
            }

            virtual double getKpWheel() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return kp_wheel_;
            }

            virtual double getKvWheel() override
            {
                ops_wbc_utils::ScopedLock lock(mutex_);
                return kv_wheel_;
            }
        
        private:
            void update(ops_wbc_robot_controller::ParamConfig& config, uint32_t level);
            using ParamConfigServer = dynamic_reconfigure::Server<ops_wbc_robot_controller::ParamConfig>;
            using ParamConfigServerPtr = std::shared_ptr<ParamConfigServer>;

            std::mutex mutex_;
            ParamConfigServerPtr server_;
            dynamic_reconfigure::Server<ops_wbc_robot_controller::ParamConfig>::CallbackType f_;

            uint32_t dof_;
            uint32_t macro_dof_;

            Eigen::MatrixXd Kp_joint_;
            Eigen::MatrixXd Kv_joint_;
            Eigen::MatrixXd Kp_task_;
            Eigen::MatrixXd Ki_task_;
            Eigen::MatrixXd Kv_task_;
            Eigen::Vector3d i_clipping_task_pos_;
            Eigen::Vector3d i_clipping_task_ori_;
            Eigen::MatrixXd Kv_damp_;
            Eigen::MatrixXd Kp_limit_;
            Eigen::MatrixXd Kv_limit_;
            double joint_error_max_;
            double pos_error_max_;
            double ori_error_max_;
            double dq_max_;
            double vx_max_;
            double kp_wheel_;
            double kv_wheel_;
            Eigen::Vector3d g_;
            Eigen::MatrixXd b_;
    };
    using ParamPtr = std::shared_ptr<Param>;
}
#endif // __OPS_WBC_ROBOT_CONTROLLER_PARAM_HPP
