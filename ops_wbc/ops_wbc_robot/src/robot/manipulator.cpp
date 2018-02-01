/*************************************************************************
	> File Name: manipulator.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 31 Jan 2018 09:33:02 PM PST
 ************************************************************************/

#include <ros/ros.h>
#include <ops_wbc_digital_filter/pseudo_differentiator.hpp>
#include <utils/exception.hpp>
#include "robot/manipulator.hpp"
#include "robot/math.hpp"
#include "definition.hpp"

using namespace ops_wbc_robot;
void Manipulator::init(uint32_t dof, const Eigen::VectorXd& init_q)
{
    if (dof != init_q.rows())
    {
        std::stringstream msg;
        msg << "dof != init_q.rows()" << std::endl
        << "  dof           : " << init_dof << std::endl
        << "  init_q.rows() : " << init_q.rows();
        throw ops_wbc_utils::Exception("Manipulator::init", msg.str());
    }

    dof_ = dof;
    q_ = Eigen::VectorXd::Zero(dof_);
    dq_ = Eigen::VectorXd::Zero(dof_);

    T_.resize(link_.size());
    for (uint32_t i = 0; i < T_.size(); i++)
    {
        T_[i] = link_[i]->T_org;
    }
    T_abs_.resize(dof_ + 1);
    for (uint32_t i = 0; i < T_.size(); i++)
    {
        T_abs_[i] = Eigen::Matrix4d::Identity();
    }
    C_abs_.resize(dof_ + 1);
    for(uint32_t i = 0; i < C_abs_.size(); ++i)
    {
        C_abs_[i] = Eigen::Matrix4d::Identity();
    }
    Pin_.resize(dof_ + 1);
    for(uint32_t i = 0; i < Pin_.size(); ++i)
    {
        Pin_[i] = Eigen::Vector3d::Zero();
    }
    
    q_ = init_q;
    pseudo_differentiator_ = std::make_shared<ops_wbc_digital_filter::PseudoDifferentiator>(update_rate_, cutoff_frequency_);
    pseudo_differentiator_->init(q_, dq_);

    J_.resize(link_.size());
    M_.resize(dof_, dof_);
    M_inv_.resize(dof_, dof_);

    for(uint32_t i = 0; i < link_.size(); ++i)
    {
        name_to_idx_[link_[i]->name] = i;
    }
}

void Manipulator::update(const Eigen::VectorXd& q_msr)
{
    if (q_msr.rows() != dof_)
    {
        std::stringstream msg;
        msg << "q_.rows() != dof" << std::endl
            << "  q_.rows   : " << q_msr.rows() << std::endl
            << "  dof : " << dof_;
        throw ops_wbc_utils::Exception("Manipulator::update", msg.str());
    }

    q_ = q_msr;
    this->computeForwardkinematics();
    this->computeVelocity();
    updated_joint_ = true;
}

void Manipulator::update(const Eigen::VectorXd& q_msr, const Eigen::VectorXd& dq_msr)
{
    if(q_msr.rows() != dof_)
    {
        std::stringstream msg;
        msg << "q_.rows() != dof" << std::endl
            << "  q_.rows   : " << q_msr.rows() << std::endl
            << "  dof : " << dof_;
        throw ops_wbc_utils::Exception("Manipulator::update", msg.str());
    }

    if(dq_msr.rows() != dof_)
    {
        std::stringstream msg;
        msg << "dq_.rows() != dof" << std::endl
            << "  dq_.rows   : " << dq_msr.rows() << std::endl
            << "  dof       : " << dof_;
        throw ops_wbc_utils::Exception("Manipulator::update", msg.str());
    }

    q_  = q_msr;
    dq_ = dq_msr;
    this->computeForwardKinematics();
    updated_joint_ = true;
}

void Manipulator::computeJacobian()
{
    if (J_.size() != link_.size())
    {
        std::stringstream msg;
        msg << "J0.size() != link_.size()" << std::endl
            << "  J0.size   : " << J_.size() << std::endl
            << "  link_.size : " << link_.size();
        throw ops_wbc_utils::Exception("Manipulator::computeJacobian", msg.str());
    }

    for(uint32_t i = 0; i < link_.size(); ++i)
    {
        this->computeJacobian(i, J_[i]);
    }
}

void Manipulator::computeMassMatrix()
{
    M_ = Eigen::MatrixXd::Zero(M_.rows(), M_cols());
    for (uint32_t i = 0; i < link_.size(); i++)
    {
        Eigen::MatrixXd Jv = J_[i].block(0, 0, 3, J_[i].cols());
        Eigen::MatrixXd Jw = J_[i].block(3, 0, 3, J_[i].cols());
        M_ += link_[i]->m * Jv.transpose() * Jv + Jw.transpose() * link_[i]->I * Jw;
    }

    if(M_.rows() > macro_dof_ && M_.cols() > macro_dof_)
    {
        // TODO : Can I really ignore this coupling !?
        Eigen::MatrixXd M_macro = Eigen::MatrixXd::Zero(macro_dof_, macro_dof_);

        for(uint32_t i = 0; i < macro_dof_; ++i)
        {
            M_macro.coeffRef(i, i) = M_.coeff(i, i);
            M_inv_.coeffRef(i, i) = 1.0 / M_.coeff(i, i);
        }
        M_.block(0, 0, macro_dof_, macro_dof_) = M_macro;

        M_.block(0, macro_dof_, macro_dof_, M_.cols() - macro_dof_) = Eigen::MatrixXd::Zero(macro_dof_, M_.cols() - macro_dof_);
        M_.block(macro_dof_, 0, M_.rows() - macro_dof_, macro_dof_) = Eigen::MatrixXd::Zero(M_.rows() - macro_dof_, macro_dof_);
    }

    else
    {
        std::stringstream msg;
        msg << "Mass matrix size is not invalid." << std::endl
            << "  macr_manipulator_dof : " << macro_dof_ << std::endl
            << "  M.rows    : " << M_.rows() << std::endl
            << "  M.cols    : " << M_.cols();
        throw ops_wbc_utils::Exception("Manipulator::computeMassMatrix", msg.str());
    }

    uint32_t mini_dof = dof_ - macro_dof_;
    M_inv_.block(macro_dof_, macro_dof_, mini_dof, mini_dof) = M_.block(macro_dof_, macro_dof_, mini_dof, mini_dof).inverse();
}

bool Manipulator::reached(const Eigen::VectorXd& qd, double threshold)
{
    if(qd.rows() != q_.rows())
    {
        std::stringstream msg;
        msg << "qd.rows() != q_.rows()" << std::endl
            << "  qd.rows() : " << qd.rows() << std::endl
            << "  q_.rows()  : " << q_.rows();
        throw ops_wbc_utils::Exception("Manipulator::reached", msg.str());
    }

    if((q_ - qd).norm() < threshold)
    {
        return true;
    }
    return false;
}

bool Manipulator::hasLink(const std::string& name)
{
    if(name_to_idx_.find(name) == name_to_idx_.end())
    {
        return false;
    }
    return true;
}

void Manipulator::addLink(const LinkPtr& link)
{
    link_.push_back(link);
}

void Manipulator::reverseLink()
{
    std::reverse(std::begin(link_), std::end(link_));
}

void Manipulator::print()
{
    std::cout << "name : " << name_ << std::endl
              << "dof : " << dof_ << std::endl
              << "q_ : " << std::endl << q_ << std::endl
              << "dq_ : " << std::endl << dq_ << std::endl;

    for(uint32_t i = 0; i < T_.size(); ++i)
    {
        std::cout << "T_[" << i << "] :" << std::endl << T_[i] << std::endl;
    }

    for(uint32_t i = 0; i < link_.size(); ++i)
    {
        link_[i]->print();
    }
}
