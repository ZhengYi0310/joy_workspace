/*************************************************************************
	> File Name: friction_compensation.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 13 Feb 2018 05:30:38 PM PST
 ************************************************************************/

#include "ops_wbc_robot_controller/friction_compensation.hpp"

using namespace ops_wbc_robot_controller;

FrictionCompensation::FrictionCompensation(const ops_wbc_robot::RobotPtr& robot)
{
  robot_ = robot;
  mnp_name_ = robot_->getManipulatorName();
  N_ = Eigen::MatrixXd::Identity(robot->getDOF(), robot->getDOF());
  b_ = Eigen::VectorXd::Zero(robot->getDOF()).asDiagonal();
}

void FrictionCompensation::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  tau = Eigen::VectorXd::Zero(robot_->getDOF());
  uint32_t macro_dof = robot_->getMacroManipulatorDOF();

  uint32_t offset = 0;
  for(uint32_t i = 0; i < mnp_name_.size(); ++i)
  {
    mnp_ = robot_->getManipulator(mnp_name_[i]);
    uint32_t mini_dof = mnp_->getDOF() - macro_dof;

    tau.block(macro_dof + offset, 0, mini_dof, 1) = b_.block(macro_dof + offset, macro_dof + offset, mini_dof, mini_dof) * mnp_->dq().block(macro_dof, 0, mini_dof, 1);

    offset += mini_dof;
  }

  tau_ = tau;
}

