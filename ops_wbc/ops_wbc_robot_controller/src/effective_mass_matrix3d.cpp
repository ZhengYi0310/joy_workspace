/*************************************************************************
	> File Name: effective_mass_matrix3d.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 12 Feb 2018 08:13:18 PM PST
 ************************************************************************/

#include<iostream>
#include "ops_wbc_robot_controller/effective_mass_matrix3d.hpp"

using namespace ops_wbc_robot_controller;

void EffectiveMassMatrix3d::compute(const Eigen::Matrix3d& lambda_inv, Eigen::Matrix3d& lambda, double thresh)
{
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(lambda_inv, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d S = svd.singularValues().asDiagonal();
    for (uint32_t i = 0; i < 3; i++)
    {
        if (S.coeff(i, i) < thresh)
        {
            S.coeffRef(i, i) = thresh;
        }
    }

    lambda = svd.matrixV() * S.inverse() * svd.matrixU().transpose();
}

