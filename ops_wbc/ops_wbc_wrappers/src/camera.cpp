/*************************************************************************
	> File Name: camera.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 24 Jan 2018 02:10:39 PM PST
 ************************************************************************/

#include <GL/gl.h>
#include <GL/glut.h>
#include <ops_wbc_wrappers/camera.hpp>
#include <ops_wbc_wrappers/quaternion.hpp>
using namespace ops_wbc_wrappers;

Camera::Camera(bool p, double fovy, double z_near, double z_far, 
               const Eigen::Vector3d& pp, const Eigen::Vector3d& cc, const Eigen::Vector3d& uu,
               double zoom_rate, double translate_rate, double rotate_rate)
                : perspective_(p), fovy_(fovy), z_near_(z_near), z_far_(z_far), left_(-1.0), right_(1.0), bottom_(1.0), top_(1.0),
                pos_(pp), center_(cc), up_(uu), pre_pos_(pp), pre_center_(cc), pre_up_(uu),
                init_pos_(pp), init_center_(cc), init_up_(uu),
                zoom_rate_(zoom_rate), translate_rate_(translate_rate), rotate_rate_(rotate_rate)
              {}

void Camera::look(int w, int h)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glViewport(0, 0, w, h);

    if (perspective_)
    {
        gluPerspective(fovy_, static_cast<double>(w) / h, z_near_, z_far_);
    }
    else
    {
        this->calcOrthoParams(w, h);
        glOrtho(left_, right_, bottom_, top_, z_near_, z_far_);
    }

    gluLookAt(pos_.coeff(0), pos_.coeff(1), pos_.coeff(2),
              center_.coeff(0), center_.coeff(1), center_.coeff(2),
              up_.coeff(0), up_.coeff(1), up_.coeff(2));
}

void Camera::reset()
{
    pos_ = pre_pos_ = init_pos_;
    center_ = pre_center_ = init_center_;
    up_ = pre_up_ = init_up_;
}

void Camera::zoomin()
{
    Eigen::Vector3d temp = pos_ - center_;

    temp *= 1.0 - zoom_rate_;
    pos_ = temp + center_;
}

void Camera::zoomout()
{
    Eigen::Vector3d temp = pos_ - center_;

    temp *= 1.0 + zoom_rate_;
    pos_ = temp + center_;
}

void Camera::translate(int dx, int dy)
{
    Eigen::Vector3d roll_axis = (pos_ - center_).cross(up_);
    roll_axis /= roll_axis.norm();

    double rate_ = (pos_ - center_).norm() * translate_rate_;

    pos_    += dx * rate_ * roll_axis;
    center_ += dx * rate_ * roll_axis;

    Eigen::Vector3d yaw_axis = roll_axis.cross(pos_ - center_);
    yaw_axis /= yaw_axis.norm();

    pos_    += dy * rate_ * yaw_axis;
    center_ += dy * rate_ * yaw_axis;
}

void Camera::rotate(int dx, int dy)
{
    Eigen::Vector3d temp_p = pos_ - center_;
    Eigen::Vector3d roll_axis = temp_p.cross(up_);
    roll_axis /= roll_axis.norm();

    Quaternion q_eye(temp_p);
    Quaternion q_up(up_);

    double roll = -dy * rotate_rate_;
    Quaternion q_roll(roll_axis, roll);

    q_eye.rotateWith(q_roll);
    q_up.rotateWith(q_roll);

    double yaw = dx * rotate_rate_;
    Eigen::Vector3d yaw_axis;
    yaw_axis << q_up.getX(), q_up.getY(), q_up.getZ();
    Quaternion q_yaw(yaw_axis, yaw);

    q_eye.rotateWith(q_yaw);

    pos_.coeffRef(0) = q_eye.getX() + center_.coeff(0);
    pos_.coeffRef(1) = q_eye.getY() + center_.coeff(1);
    pos_.coeffRef(2) = q_eye.getZ() + center_.coeff(2);

    up_.coeffRef(0) = q_up.getX();
    up_.coeffRef(1) = q_up.getY();
    up_.coeffRef(2) = q_up.getZ();
}

void Camera::calcOrthoParams(int w, int h)
{
    double norm = (pos_ - center_).norm();
    double d = 2.0 * atan(fovy_ * 0.5) * norm;
    double view_h = d;
    double view_w = d;

    if (h < w)
    {
        view_h = view_w * (static_cast<double>(h) / w);
    }
    else
    {
        view_w = view_h * (static_cast<double>(w) / h);
    }

    right_ = view_w * 0.5;
    left_ = -right_;
    top_ = view_h * 0.5;
    bottom_ = -top_;
}




