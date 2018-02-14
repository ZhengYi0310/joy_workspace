/*************************************************************************
	> File Name: joint_control.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 13 Feb 2018 08:48:14 PM PST
 ************************************************************************/
#include "utils/exception.hpp"
#include "ops_wbc_robot_controller/joint_control.hpp"

using namespace ops_wbc_robot_controller;

JointControl::JointControl(const ops_wbc_robot::ManipulatorPtr& mnp)
{
    mnp_ = mnp;
    N_ = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());
    dqd_exist_ = false;
}

void JointControl::setGoal(const Eigen::MatrixXd& qd)
{
    if(qd.rows() != mnp_->getDOF())
    {
        std::stringstream msg;
        msg << "qd.rows() != mnp_->getDOF()" << std::endl
            << "  qd.rows   : " << qd.rows() << std::endl
            << "  mnp_->dof : " << mnp_->getDOF();
        throw ops_wbc_utils::Exception("JointControl::setGoal", msg.str());
    }
    dqd_exist_ = false;
    qd_ = qd.block(0, 0, qd.rows(), 1);
}

void JointControl::setGoal(const Eigen::MatrixXd& qd, const Eigen::MatrixXd& dqd)
{
    if(qd.rows() != mnp_->getDOF())
    {
        std::stringstream msg;
        msg << "qd.rows() != mnp_->getDOF()" << std::endl
            << "  qd.rows   : " << qd.rows() << std::endl
            << "  mnp_->dof : " << mnp_->getDOF();
        throw ops_wbc_utils::Exception("JointControl::setGoal", msg.str());
    }
    
    if(dqd.rows() != mnp_->getDOF())
    {
        std::stringstream msg;
        msg << "dqd.rows() != mnp_->getDOF()" << std::endl
            << "  dqd.rows   : " << dqd.rows() << std::endl
            << "  mnp_->dof : " << mnp_->getDOF();
        throw ops_wbc_utils::Exception("JointControl::setGoal", msg.str());
    }

    qd_ = qd.block(0, 0, qd.rows(), 1);
    dqd_ = qd.block(0, 0, dqd.rows(), 1);
    dqd_exist_ = true;
}

void JointControl::computeGeneralizedForce(Eigen::VectorXd& tau)
{
    tau = Eigen::VectorXd::Zero(mnp_->getDOF());

    Eigen::VectorXd error = qd_ - mnp_->q();
    Eigen::MatrixXd Kpv = param_->getKpJoint().block(0, 0, mnp_->getDOF(), mnp_->getDOF());

    for(uint32_t i = 0; i < Kpv.rows(); ++i)
    {
        Kpv.coeffRef(i, i) /= param_->getKvJoint().block(0, 0, mnp_->getDOF(), mnp_->getDOF()).coeff(i, i);
    }

    if (!dqd_exist_)
    {
        Eigen::VectorXd dqd = Kpv * error;

        //const double dq_max = 25.0;

        for(uint32_t i = 0; i < dqd.rows(); ++i)
        {
            if(dqd[i] < -param_->getDqMax())
            {
                dqd[i] = -param_->getDqMax();
            }
            else if(dqd[i] > param_->getDqMax())
            {
                dqd[i] = param_->getDqMax();
            }
        }
        Eigen::VectorXd tau_unit = -param_->getKvJoint().block(0, 0, mnp_->getDOF(), mnp_->getDOF()) * (mnp_->dq() - dqd);

        tau = tau_ = mnp_->getMassMatrix() * tau_unit;
    }
    else 
    {
        error *= Kpv;
        for(uint32_t i = 0; i < dqd_.rows(); ++i)
        {
            if(dqd_[i] < -param_->getDqMax())
            {
                dqd_[i] = -param_->getDqMax();
            }
            else if(dqd_[i] > param_->getDqMax())
            {
                dqd_[i] = param_->getDqMax();
            }
        }
        Eigen::VectorXd tau_unit = -param_->getKvJoint().block(0, 0, mnp_->getDOF(), mnp_->getDOF()) * ((mnp_->dq() - dqd_) - error);

        tau = tau_ = mnp_->getMassMatrix() * tau_unit;
    }
}
