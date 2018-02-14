/*************************************************************************
	> File Name: joint_limit.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 13 Feb 2018 06:02:14 PM PST
 ************************************************************************/

#include "ops_wbc_robot_controller/joint_limit.hpp"
using namespace ops_wbc_robot_controller;

JointLimit::JointLimit(const ops_wbc_robot::ManipulatorPtr& mnp, double threshold) : threshold_(threshold)
{
    mnp_ = mnp;
    tau_ = Eigen::VectorXd::Zero(mnp_->getLinkNum());

    q_max_ = Eigen::VectorXd::Zero(mnp_->getLinkNum());
    q_min_ = Eigen::VectorXd::Zero(mnp_->getLinkNum());

    lock_.resize(mnp_->getLinkNum());

    for (uint32_t i = mnp_->getMacroManipulatorDOF(); i < mnp_->getLinkNum(); i++)
    {
        q_max_[i] = mnp_->getLink(i)->q_max;
        q_min_[i] = mnp_->getLink(i)->q_min;
        lock_[i] = false;

    }

    N_ = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());
}

