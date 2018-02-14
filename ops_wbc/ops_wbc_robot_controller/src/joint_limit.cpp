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

void JointLimit::computeGeneralizedForce(Eigen::VectorXd& tau)
{
    tau = Eigen::VectorXd::Zero(mnp_->getDOF());
    N_ = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());

    for (uint32_t i = mnp_->getMacroManipulatorDOF(); i < mnp_->q().rows(); i++)
    {
        if (q_min_[i] == q_max_[i]) continue;

        double q = mnp_->q()[i];
        double max = q_max_[i];
        double min = q_min_[i];

        Eigen::VectorXd tau_damp = Eigen::VectorXd::Zero(mnp_->getDOF());
        tau_damp = -mnp_->getMassMatrix() * param_->getKvDamp().coeff(0, 0) * mnp_->dq();

        if (lock_[i])
        {
            if (moveAwayFromMax(mnp_->q()[i], q_max_[i]) &&
                moveAwayFromMin(mnp_->q()[i], q_min_[i]))
            {
                lock_[i] = false;
                N_.coeffRef(i, i) = 1.0;
            }
        }

        if (max - threshold_ < q && q < max)
        {
            //std::cout << i << " locked" << std::endl;
            tau[i] = -param_->getKpLimit().coeff(i, i) * (max - q) + tau_damp[i];
            N_.coeffRef(i, i) = 0.0;
            lock_[i] = true;
        }

        else if (max <= q)
        {
            tau[i] = -param_->getKpLimit().coeff(i, i) * threshold_ + tau_damp[i];
            N_.coeffRef(i, i) = 0.0;
            lock_[i] = true;
        }
        else if (min < q < min + threshold_)
        {
            tau[i] = -param_->getKpLimit().coeff(i, i) * (min - q) + tau_damp[i];
            N_.coeffRef(i, i) = 0.0;
            lock_[i] = true;
        }
        else if (q <= min)
        {
            tau[i] = param_->getKpLimit().coeff(i, i) * threshold_ + tau_damp[i];
            N_.coeffRef(i, i) = 0.0;
            lock_[i] = true;
        }
    }
}

bool JointLimit::moveAwayFromMax(double q, double max)
{
    if (q < max - 2.0 * threshold_)
        return true;
    return false;
}

bool JointLimit::moveAwayFromMin(double q, double min)
{
    if (q > min + 2.0 * threshold_)
        return true;
    return false;
}

