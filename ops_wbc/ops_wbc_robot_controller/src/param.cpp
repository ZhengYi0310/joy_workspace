/*************************************************************************
	> File Name: param.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 05 Feb 2018 07:58:54 PM PST
 ************************************************************************/

#include <ops_wbc_robot_controller/param.hpp>

using namespace ops_wbc_robot_controller;

Param::Param(const ops_wbc_robot::RobotPtr& robot)
    : dof_(robot->getDOF()), macro_dof_(robot->getMacroManipulatorDOF()), 
      joint_error_max_(100.0), pos_error_max_(100.0), ori_error_max_(100.0),
      dq_max_(1.0), vx_max_(0.5), kp_wheel_(0.0), kv_wheel_(0.0)
    {
        Kp_joint_ = Eigen::MatrixXd::Zero(dof_, dof_);
        Kv_joint_ = Eigen::MatrixXd::Zero(dof_, dof_);
        Kp_task_  = Eigen::MatrixXd::Zero(6, 6);
        Ki_task_  = Eigen::MatrixXd::Zero(6, 6);
        Kv_task_  = Eigen::MatrixXd::Zero(6, 6);
        i_clipping_task_pos_ = Eigen::Vector3d::Zero();
        i_clipping_task_ori_ = Eigen::Vector3d::Zero();
        Kv_damp_  = Eigen::MatrixXd::Zero(dof_, dof_);
        Kp_limit_ = Eigen::MatrixXd::Zero(dof_, dof_);
        Kv_limit_ = Eigen::MatrixXd::Zero(dof_, dof_);

        g_ << 0.0, 0.0, -9.80665;
        b_ = Eigen::MatrixXd::Zero(dof_, dof_);
        ros::NodeHandle local_nh("~/ops_wbc_robot_controller");
        f_ = boost::bind(&Param::update, this, _1, _2);
        server_ = std::make_shared<ParamConfigServer>(local_nh);
        server_->setCallback(f_);
    }

void Param::update(ops_wbc_robot_controller::ParamConfig& config, uint32_t level)
{
    ops_wbc_utils::ScopedLock lock(mutex_);

    Eigen::VectorXd Kp_joint = Eigen::VectorXd::Constant(dof_, config.kp_joint);
    Eigen::VectorXd Kv_joint = Eigen::VectorXd::Constant(dof_, config.kv_joint);
    Kp_joint.block(0, 0, macro_dof_, 1) = Eigen::VectorXd::Constant(macro_dof_, config.kp_joint_macro);
    Kv_joint.block(0, 0, macro_dof_, 1) = Eigen::VectorXd::Constant(macro_dof_, config.kv_joint_macro);
    Eigen::VectorXd Kp_task_pos = Eigen::VectorXd::Constant(3, config.kp_task_pos);
    Eigen::VectorXd Ki_task_pos = Eigen::VectorXd::Constant(3, config.ki_task_pos);
    Eigen::VectorXd Kv_task_pos = Eigen::VectorXd::Constant(3, config.kv_task_pos);
    Eigen::VectorXd Kp_task_ori = Eigen::VectorXd::Constant(3, config.kp_task_ori);
    Eigen::VectorXd Ki_task_ori = Eigen::VectorXd::Constant(3, config.ki_task_ori);
    Eigen::VectorXd Kv_task_ori = Eigen::VectorXd::Constant(3, config.kv_task_ori);
    Eigen::VectorXd Kv_damp  = Eigen::VectorXd::Constant(dof_, config.kv_damp);
    Eigen::VectorXd Kp_limit = Eigen::VectorXd::Constant(dof_, config.kp_limit);
    Eigen::VectorXd Kv_limit = Eigen::VectorXd::Constant(dof_, config.kv_limit);

    Kp_joint_ = Kp_joint.asDiagonal();
    Kv_joint_ = Kv_joint.asDiagonal();
    Kp_task_.block(0, 0, 3, 3) = Kp_task_pos.asDiagonal();
    Ki_task_.block(0, 0, 3, 3) = Ki_task_pos.asDiagonal();
    Kv_task_.block(0, 0, 3, 3) = Kv_task_pos.asDiagonal();

    Kp_task_.block(3, 3, 3, 3) = Kp_task_ori.asDiagonal();
    Ki_task_.block(3, 3, 3, 3) = Ki_task_ori.asDiagonal();
    Kv_task_.block(3, 3, 3, 3) = Kv_task_ori.asDiagonal();

    Kv_damp_ = Kv_damp.asDiagonal();
    Kp_limit_ = Kp_limit.asDiagonal();
    Kv_limit_ = Kv_limit.asDiagonal();

    joint_error_max_ = config.joint_error_max;
    pos_error_max_ = config.pos_error_max;
    ori_error_max_ = config.ori_error_max;
    dq_max_ = config.dq_max;
    vx_max_ = config.vx_max;

    g_ << config.gx, config.gy, config.gz;
    b_ = Eigen::VectorXd::Constant(dof_, config.b).asDiagonal();

    i_clipping_task_pos_ = Eigen::Vector3d::Constant(config.i_clipping_task_pos);
    i_clipping_task_ori_ = Eigen::Vector3d::Constant(config.i_clipping_task_ori);

    kp_wheel_ = config.kp_wheel;
    kv_wheel_ = config.kv_wheel;

}

