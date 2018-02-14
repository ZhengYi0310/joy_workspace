/*************************************************************************
	> File Name: orientation_control.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 13 Feb 2018 10:51:17 PM PST
 ************************************************************************/

#include "utils/exception.hpp"
#include "ops_wbc_robot_controller/effective_mass_matrix3d.hpp"
#include "ops_wbc_robot_controller/orientation_control.hpp"

using namespace ops_wbc_robot_controller;

OrientationControl::OrientationControl(const ops_wbc_robot::ManipulatorPtr& mnp, const std::string& target_link, double eigen_thresh)
  : updated_(false), target_link_(target_link), eigen_thresh_(eigen_thresh)
{
    mnp_ = mnp;

    if(!mnp_->hasLink(target_link))
    {
        std::stringstream msg;
        msg << target_link << " was not found in mnp_->name_to_idx.";
        throw ops_wbc_utils::Exception("OrientationControl::OrientationControl", msg.str());
    }

    idx_ = mnp_->getIndex(target_link);
    I_ = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());
    N_ = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());
    M_unit_ = Eigen::Vector3d::Zero();
    dRd_exist_ = false;
}

void OrientationControl::setGoal(const Eigen::MatrixXd& Rd, const Eigen::MatrixXd& dRd)
{
    if(Rd.rows() != 3)
    {
        std::stringstream msg;
        msg << "Rd.rows() != 3" << std::endl
            << "  Rd.rows : " << Rd.rows();
        throw ops_wbc_utils::Exception("OrientationControl::setGoal", msg.str());
    }
    if(Rd.cols() != 3)
    {
        std::stringstream msg;
        msg << "Rd.cols() != 3" << std::endl
            << "  Rd.cols : " << Rd.cols();
        throw ops_wbc_utils::Exception("OrientationControl::setGoal", msg.str());
    }

    if(dRd.rows() != 3)
    {
        std::stringstream msg;
        msg << "dRd.rows() != 3" << std::endl
            << "  dRd.rows : " << dRd.rows();
        throw ops_wbc_utils::Exception("OrientationControl::setGoal", msg.str());
    }
    if(dRd.cols() != 3)
    {
        std::stringstream msg;
        msg << "dRd.cols() != 3" << std::endl
            << "  dRd.cols : " << dRd.cols();
        throw ops_wbc_utils::Exception("OrientationControl::setGoal", msg.str());
    }
    
    dRd_ = dRd;
    Rd_ = Rd;
    dRd_exist_ = true;
}

void OrientationControl::updateModel()
{
    Jw_ = mnp_->getJacobian()[idx_].block(3, 0, 3, mnp_->getJacobian()[idx_].cols());
    lambda_inv_ = Jw_ * mnp_->getMassMatrixInv() * Jw_.transpose();
    EffectiveMassMatrix3d::compute(lambda_inv_, lambda_, eigen_thresh_);
    J_dyn_inv_ = mnp_->getMassMatrixInv() * Jw_.transpose() * lambda_;
    N_ = I_ - Jw_.transpose() * J_dyn_inv_.transpose();

    updated_ = true;
}

void OrientationControl::computeGeneralizedForce(Eigen::VectorXd& tau)
{
    if(!updated_)
    {
        tau = Eigen::VectorXd::Zero(mnp_->getDOF());
        return;
    }

    Eigen::Matrix3d R = mnp_->getTransformAbs(idx_).block(0, 0, 3, 3);
    Eigen::Quaternion<double> q;
    q = R * Rd_.inverse();
    double norm = sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
    Eigen::Vector3d del_phi;
    double c = 0.0;
    if(norm != 0.0)
    {
        c = 2.0 * acos(q.w()) / norm;

        if(c > param_->getOriErrorMax())
            c = param_->getOriErrorMax();
        else if(c < -param_->getOriErrorMax())
            c = -param_->getOriErrorMax();
    }
    del_phi << q.x() * c, q.y() * c, q.z() * c;

    M_unit_ = -param_->getKpTask().block(3, 3, 3, 3) * del_phi -param_->getKvTask().block(3, 3, 3, 3) * Jw_ * mnp_->dq();
    Eigen::Vector3d M = lambda_ * M_unit_;
    tau = tau_ = Jw_.transpose() * M;
}

