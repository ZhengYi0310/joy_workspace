/*************************************************************************
	> File Name: position_control.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 13 Feb 2018 09:54:28 PM PST
 ************************************************************************/

#include <utils/exception.hpp>
#include "ops_wbc_robot_controller/effective_mass_matrix3d.hpp"
#include "ops_wbc_robot_controller/position_control.hpp"

using namespace ops_wbc_robot_controller;

PositionControl::PositionControl(const ops_wbc_robot::ManipulatorPtr& mnp, const std::string& target_link, double eigen_thresh)
  : updated_(false), target_link_(target_link), eigen_thresh_(eigen_thresh)
{
    mnp_ = mnp;

    if(!mnp_->hasLink(target_link))
    {
        std::stringstream msg;
        msg << target_link << " was not found in mnp_->name_to_idx.";
        throw ops_wbc_utils::Exception("PositionControl::PositionControl", msg.str()); 
    }

    idx_ = mnp_->getIndex(target_link);
    I_ = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());
    N_ = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());
    F_unit_ = Eigen::Vector3d::Zero();
    error_sum_ = Eigen::Vector3d::Zero();
    dxd_exist_ =false;
}

void PositionControl::setGoal(const Eigen::MatrixXd& xd)
{
    if(xd.rows() != 3)
    {
        std::stringstream msg;
        msg << "xd.rows() != 3" << std::endl
            << "  xd.rows : " << xd.rows();
        throw ops_wbc_utils::Exception("PositionControl::setGoal", msg.str());
    }

    xd_ = xd.block(0, 0, xd.rows(), 1);
    dxd_exist_ = false;
}

void PositionControl::setGoal(const Eigen::MatrixXd& xd, const Eigen::MatrixXd& dxd)
{
    if(xd.rows() != 3)
    {
        std::stringstream msg;
        msg << "xd.rows() != 3" << std::endl
            << "  xd.rows : " << xd.rows();
        throw ops_wbc_utils::Exception("PositionControl::setGoal", msg.str());
    }

    if(dxd.rows() != 3)
    {
        std::stringstream msg;
        msg << "xd.rows() != 3" << std::endl
            << "  xd.rows : " << xd.rows();
        throw ops_wbc_utils::Exception("PositionControl::setGoal", msg.str());
    }
    

    xd_ = xd.block(0, 0, xd.rows(), 1);
    dxd_ = dxd.block(0, 0, dxd.rows(), 1);
    dxd_exist_ = true;
}

void PositionControl::updateModel()
{
    Jv_ = mnp_->getJacobian()[idx_].block(0, 0, 3, mnp_->getJacobian()[idx_].cols());
    lambda_inv_ = Jv_ * mnp_->getMassMatrixInv() * Jv_.transpose();
    EffectiveMassMatrix3d::compute(lambda_inv_, lambda_, eigen_thresh_);
    J_dyn_inv_ = mnp_->getMassMatrixInv() * Jv_.transpose() * lambda_;
    N_ = I_ - Jv_.transpose() * J_dyn_inv_.transpose();
    updated_ = true;

}

void PositionControl::computeGeneralizedForce(Eigen::VectorXd& tau)
{
    if(!updated_)
    {
        tau = Eigen::VectorXd::Zero(mnp_->getDOF());
        return;
    }

    Eigen::Vector3d x = mnp_->getTransformAbs(idx_).block(0, 3, 3, 1);
    Eigen::Vector3d error = xd_ - x; 

    Eigen::Matrix3d Kpv = param_->getKpTask().block(0, 0, 3, 3);

    for(uint32_t i = 0; i < Kpv.rows(); ++i)
    {
        Kpv.coeffRef(i, i) /= param_->getKvTask().block(0, 0, 3, 3).coeff(i, i);
    }

    error_sum_ += error;
    for(uint32_t i = 0; i < error_sum_.rows(); ++i)
    {
        if(error_sum_[i] > param_->getIClippingTaskPos()[i])
            error_sum_[i] = param_->getIClippingTaskPos()[i];
        else if(error_sum_[i] < -param_->getIClippingTaskPos()[i])
            error_sum_[i] = -param_->getIClippingTaskPos()[i];
    }

    Eigen::Vector3d dxd = Kpv * error; //+ param_->getKiTask().block(0, 0, 3, 3) * error_sum_;

    if (dxd.norm() > param_->getVxMax())
    {
        dxd = param_->getVxMax() / dxd.norm() * dxd; 
    }

    F_unit_ = -param_->getKvTask().block(0, 0, 3, 3) * (Jv_ * mnp_->dq() - dxd);
    Eigen::Vector3d F = lambda_ * F_unit_;
    tau = tau_ = Jv_.transpose() * F;
}





