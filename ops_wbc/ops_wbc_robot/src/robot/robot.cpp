/*************************************************************************
	> File Name: robot.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Sun 04 Feb 2018 03:15:52 PM PST
 ************************************************************************/

#include <utils/exception.hpp>
#include "ops_wbc_robot/robot/robot.hpp"
using namespace ops_wbc_robot;

void Robot::computeJacobian()
{
    for (uint32_t i = 0; i < mnp_name_.size(); i++)
    {
        if(mnp_.find(mnp_name_[i]) == mnp_.end())
        {
            std::stringstream msg;
            msg << "Could not find manipulator : " << mnp_name_[i];
            throw ops_wbc_utils::Exception("Robot::computeJacobian", msg.str());
        }

        mnp_[mnp_name_[i]]->computeJacobian();
    }
}

void Robot::computeJacobian(const std::string& mnp_name)
{
     if(mnp_.find(mnp_name) == mnp_.end())
    {
        std::stringstream msg;
        msg << "Could not find manipulator : " << mnp_name;
        throw ops_wbc_utils::Exception("Robot::computeJacobian", msg.str());
    }

    mnp_[mnp_name]->computeJacobian();
}

void Robot::computeMassMatrix()
{
    for (uint32_t i = 0; i < mnp_name_.size(); i++)
    {
        if (mnp_.find(mnp_name_[i]) == mnp_.end())
        {
            std::stringstream msg;
            msg << "Could not find manipulator : " << mnp_name_[i];
            throw ops_wbc_utils::Exception("Robot::computeMassMatrix", msg.str());
        }
        mnp_[mnp_name_[i]]->computeMassMatrix();
    }
    M_ = M_inv_ = Eigen::MatrixXd::Zero(dof_, dof_);
    M_.block(0, 0, macro_dof_, macro_dof_) = std::begin(mnp_)->second->getMassMatrix().block(0, 0, 
                                                                                             macro_dof_, macro_dof_);
    M_inv_.block(0, 0, macro_dof_, macro_dof_) = std::begin(mnp_)->second->getMassMatrixInv().block(0, 0, 
                                                                                                    macro_dof_, macro_dof_);

    uint32_t offset = macro_dof_;
    for (uint32_t i = 0; i < mnp_name_.size(); i++)
    {
        ManipulatorPtr mnp = mnp_[mnp_name_[i]];
        uint32_t mini_dof = mnp->getDOF() - macro_dof_;
        M_.block(offset, offset, mini_dof, mini_dof) = mnp->getMassMatrix().block(macro_dof_, macro_dof_, mini_dof, mini_dof);
        M_inv_.block(offset, offset, mini_dof, mini_dof) = mnp->getMassMatrixInv().block(macro_dof_, macro_dof_, mini_dof, mini_dof);

        offset += mini_dof;
    }
}

void Robot::update(const Eigen::VectorXd& q)
{
    if(q.rows() != dof_)
    {
        std::stringstream msg;
        msg << "q.rows() != dof_" << std::endl
            << "  q.rows : " << q.rows() << std::endl
            << "  dof    : " << dof_;
        throw ops_wbc_utils::Exception("Robot::update", msg.str());
    }

    q_ = q;
    dq_ = Eigen::VectorXd::Zero(dof_);
    dq_.block(0, 0, macro_dof_, 1) = std::begin(mnp_)->second->dq().block(0, 0, macro_dof_, 1);
    Eigen::VectorXd q_macro = q.block(0, 0, macro_dof_, 1);
    int32_t offset = macro_dof_;
    for (uint32_t i = 0; i < mnp_name_.size(); i++)
    {
        if (mnp_.find(mnp_name_[i]) == mnp_.end())
        {
            std::stringstream msg;
            msg << "Could not find manipulator : " << mnp_name_[i];
            throw ops_wbc_utils::Exception("Robot::update", msg.str());
        }
        ManipulatorPtr mnp = mnp_[mnp_name_[i]];
        Eigen::VectorXd q_mnp = Eigen::VectorXd::Zero(mnp->getDOF());

        uint32_t mini_dof = mnp->getDOF() - macro_dof_;
        q_mnp.block(0, 0, macro_dof_, 1) = q_macro;
        q_mnp.block(macro_dof_, 0, mini_dof, 1) = q.block(offset, 0, mini_dof, 1);
        mnp->update(q_mnp);
        dq_.block(offset, 0, mini_dof, 1) = mnp->dq().block(macro_dof_, 0, mini_dof, 1);

        offset += mini_dof;
    }
}

bool Robot::reached(const std::string& mnp_name, const Eigen::VectorXd& qd, double threshold)
{
    if(mnp_.find(mnp_name) == mnp_.end())
    {
        std::stringstream msg;
        msg << "Could not find manipulator : " << mnp_name;
        throw ops_wbc_utils::Exception("Robot::reached", msg.str());
    }

    return mnp_[mnp_name]->reached(qd, threshold);
}

void Robot::add(const ManipulatorPtr& mnp)
{
    mnp_[mnp->getName()] = mnp;
    mnp_name_.push_back(mnp->getName());
}

const Eigen::MatrixXd& Robot::getJacobian(const std::string& mnp_name)
{
    if (mnp_.find(mnp_name) == mnp_.end())
    {
        std::stringstream msg;
        msg << "Could not find manipulator : " << mnp_name;
        throw ops_wbc_utils::Exception("Robot::getJacobian", msg.str());
    }

    std::string link_name = mnp_[mnp_name]->getLink(mnp_[mnp_name]->getLinkNum() - 1)->name;
    return this->getJacobian(mnp_name, link_name);
}

const Eigen::MatrixXd& Robot::getJacobian(const std::string& mnp_name, const std::string& link_name)
{
    if(mnp_.find(mnp_name) == mnp_.end())
    {
        std::stringstream msg;
        msg << "Could not find manipulator : " << mnp_name;
        throw ops_wbc_utils::Exception("Robot::getJacobian", msg.str());
    }

    if(!mnp_[mnp_name]->hasLink(link_name))
    {
        std::stringstream msg;
        msg << "Could not find name_to_idx." << std::endl
            << "  Manipulator : " << mnp_name << std::endl
            << "  Link        : " << link_name;
        throw ops_wbc_utils::Exception("Robot::getJacobian", msg.str());
    }
    int32_t idx = mnp_[mnp_name]->getIndex(link_name);
    return mnp_[mnp_name]->getJacobian()[idx];
}

const Eigen::MatrixXd& Robot::getMassMatrix(const std::string& mnp_name)
{
    if(mnp_.find(mnp_name) == mnp_.end())
    {
        std::stringstream msg;
        msg << "Could not find manipulator : " << mnp_name;
        throw ops_wbc_utils::Exception("Robot::getMassMatrix", msg.str());
  }

  return mnp_[mnp_name]->getMassMatrix();
}

const Eigen::MatrixXd& Robot::getMassMatrixInv(const std::string& mnp_name)
{
    if(mnp_.find(mnp_name) == mnp_.end())
    {
        std::stringstream msg;
        msg << "Could not find manipulator : " << mnp_name;
        throw ops_wbc_utils::Exception("Robot::getMassMatrixInv", msg.str());
    }

    return mnp_[mnp_name]->getMassMatrixInv();
}

const Eigen::VectorXd& Robot::getJointPosition(const std::string& mnp_name)
{
    if(mnp_.find(mnp_name) == mnp_.end())
    {
        std::stringstream msg;
        msg << "Could not find manipulator : " << mnp_name;
        throw ops_wbc_utils::Exception("Robot::getJointPosition", msg.str());
    }

    return mnp_[mnp_name]->q();
}

const Eigen::VectorXd& Robot::getJointVelocity(const std::string& mnp_name)
{
    if(mnp_.find(mnp_name) == mnp_.end())
    {
        std::stringstream msg;
        msg << "Could not find manipulator : " << mnp_name;
        throw ops_wbc_utils::Exception("Robot::getJointVelocity", msg.str());
    }

    return mnp_[mnp_name]->dq();
}

const uint32_t Robot::getDOF(const std::string& mnp_name)
{
    if(mnp_.find(mnp_name) == mnp_.end())
    {
        std::stringstream msg;
        msg << "Could not find manipulator : " << mnp_name;
        throw ops_wbc_utils::Exception("Robot::getDOF", msg.str());
    }

    return mnp_[mnp_name]->getDOF();
}



