/*************************************************************************
	> File Name: damping.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 13 Feb 2018 05:45:03 PM PST
 ************************************************************************/

#include<iostream>
#include "ops_wbc_robot_controller/damping.hpp"

using namespace ops_wbc_robot_controller;

Damping::Damping(const ops_wbc_robot::RobotPtr& robot)
{
    robot_ = robot;
    N_ = Eigen::MatrixXd::Identity(robot_->getDOF(), robot_->getDOF());
}

void Damping::computeGeneralizedForce(Eigen::VectorXd& tau)
{
    tau = -robot_->getMassMatrix() * param_->getKvDamp() * robot_->getJointVelocity();
    tau_ = tau;
}

