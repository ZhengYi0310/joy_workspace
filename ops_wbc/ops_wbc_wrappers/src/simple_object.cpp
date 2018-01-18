/*************************************************************************
	> File Name: simple_object.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 17 Jan 2018 09:47:28 PM PST
 ************************************************************************/

#include <ops_wbc_wrappers/simple_object.hpp>

using namespace ops_wbc_wrappers;

SimpleObject::SimpleObject() : pos_(Eigen::Vector3d::Zero()), euler_zyx_(Eigen::Vector3d::Zero()), r_(0), g_(0), b_(0)
{}

void SimpleObject::display()
{
    glLoadIdentity();
    glPushMatrix();

    glTranslated(pos_.coeff(0), pos_.coeff(1), pos_.coeff(2));
    glRotated(euler_zyx_.coeff(0), 0.0, 0.0, 1.0);
    glRotated(euler_zyx_.coeff(1), 0.0, 1.0, 0.0);
    glRotated(euler_zyx_.coeff(2), 1.0, 0.0, 0.0);

    glColor3d(static_cast<double>(r_ / 255.0), static_cast<double>(g_ / 255.0), static_cast<double>(b_ / 255.0));
    this->displayImpl();

    glPopMatrix();
}

void SimpleObject::setColor(int r, int g, int b)
{
    r_ = r;
    g_ = g;
    b_ = b;
}

void SimpleObject::setPosition(double x, double y, double z)
{
    pos_.coeffRef(0) = x;
    pos_.coeffRef(1) = y;
    pos_.coeffRef(2) = z;
}

void SimpleObject::setPosition(const Eigen::Vector3d& pos)
{
    pos_ = pos;
}

void SimpleObject::setEulerZYX(const Eigen::Vector3d& euler_zyx)
{
    euler_zyx_ = euler_zyx;
}

void SimpleObject::setEulerZYX(double rz, double ry, double rx)
{
    euler_zyx_.coeffRef(0) = rz;
    euler_zyx_.coeffRef(1) = ry;
    euler_zyx_.coeffRef(2) = rx;
}


