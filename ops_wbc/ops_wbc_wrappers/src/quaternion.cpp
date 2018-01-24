/*************************************************************************
	> File Name: quaternion.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 24 Jan 2018 02:46:23 PM PST
 ************************************************************************/

#include <ops_wbc_wrappers/exceptions.hpp>
#include <ops_wbc_wrappers/quaternion.hpp>

using namespace ops_wbc_wrappers;

Quaternion::Quaternion() : x_(0.0), y_(0.0), z_(1.0), w_(0.0)
{
}

Quaternion::Quaternion(double x, double y, double z, double w) : x_(x), y_(y), z_(z), w_(w)
{
}

Quaternion::Quaternion(const Eigen::Vector3d& axis, double angle)
{
    if(axis.norm() == 0.0)
    {
        throw Exception("Quaternion::Quaternion", "Norm of axis is zero.");
    }

    double inv_norm = 1.0 / axis.norm();
    double sin_val = sin(0.5 * angle);

    x_ = axis.coeff(0) * inv_norm * sin_val;
    y_ = axis.coeff(1) * inv_norm * sin_val;
    z_ = axis.coeff(2) * inv_norm * sin_val;
    w_ = cos(0.5 * angle);
}

Quaternion::Quaternion(const Eigen::Vector3d& pos) : x_(pos.coeff(0)), y_(pos.coeff(1)), z_(pos.coeff(2)), w_(0.0)
{
}

Quaternion& Quaternion::operator=(const Quaternion& q)
{
    if (this == &q)
        return *this;

    x_ = q.getX();
    y_ = q.getY();
    z_ = q.getZ();
    w_ = q.getW();

    return *this;
}

void Quaternion::operator*=(Quaternion& q)
{
    double x = x_;
    double y = y_;
    double z = z_;
    double w = w_;

    w_ = w * q.getW() - x * q.getX() - y * q.getY() - z * q.getZ();
    x_ = w * q.getX() + x * q.getW() + y * q.getZ() - z * q.getY();
    y_ = w * q.getY() + y * q.getW() + z * q.getX() - x * q.getZ();
    z_ = w * q.getZ() + z * q.getW() + x * q.getY() - y * q.getX();
}

const Quaternion Quaternion::operator*(Quaternion& q) const 
{
    Quaternion dst(*this);
    dst *= q;
    return dst;
}

Quaternion Quaternion::inverse()
{
    double norm = this->getNorm();
    if(norm == 0.0)
    {
        throw Exception("Quaternion::inverse", "Could not calculate inverse because the norm is zero.");
    }

    double inv_norm = 1.0 / norm;

    Quaternion q(-x_ * inv_norm, -y_ * inv_norm, -z_ * inv_norm, w_ * inv_norm);
    return q;
}

Quaternion& Quaternion::rotateWith(Quaternion& q)
{
    *this = q.inverse() * (*this) * q;
    return *this;
}

