/*************************************************************************
	> File Name: gravity_compensation.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 12 Feb 2018 09:23:16 PM PST
 ************************************************************************/

#include<iostream>
#include "ops_wbc_robot_controller/gravity_compensation.hpp"
using namespace ops_wbc_robot_controller;

GravityCompensation::GravityCompensation(const ops_wbc_robot::RobotPtr& robot)
{
    robot_ = robot;
    mnp_name_ = robot_->getManipulatorName();
    N_ = Eigen::MatrixXd::Identity(robot->getDOF(), robot->getDOF());
}

void GravityCompensation::computeGeneralizedForce(Eigen::VectorXd& tau)
{
    tau = Eigen::VectorXd::Zero(robot_->getDOF());
    uint32_t macro_dof = robot_->getMacroManipulatorDOF();

    mnp_ = std::begin(robot_->getManipulator())->second;
    for (uint32_t i = 0; i < macro_dof; i++)
    {
        tau.block(0, 0, macro_dof, 1) -= mnp_->getLink(i)->m * 
                                         mnp_->getJacobian()[i].block(0, 0, 3, macro_dof).transpose() * 
                                         param_->getG();
    }
    uint32_t offset = 0;

    for (uint32_t i = 0; i < mnp_name_.size(); i++)
    {
        mnp_ = robot_->getManipulator(mnp_name_[i]);
        uint32_t mini_dof = mnp_->getDOF() - macro_dof;

        for(uint32_t j = 0; j < mini_dof; ++j)
        {
            tau.block(macro_dof + offset, 0, mini_dof, 1) -= mnp_->getLink(j + macro_dof)->m * mnp_->getJacobian()[j + macro_dof].block(0, macro_dof, 3, mini_dof).transpose() * param_->getG();
        }

        offset += mini_dof;
    }
    tau_ = tau;
}
