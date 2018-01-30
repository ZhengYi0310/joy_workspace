/*************************************************************************
	> File Name: math.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 29 Jan 2018 08:27:38 PM PST
 ************************************************************************/

#include<iostream>
#include <ops_wbc_robot/robot/math.hpp>

namespace ops_wbc_robot
{
    namespace math 
    {
        void computeErFromQuaternion(const Eigen::Quaternion<double>& q, Eigen::MatrixXd& Er)
        {
            Er.resize(3, 4);
            Er << -q.x(),  q.w(), -q.z(),   q.y(),
                  -q.y(),  q.z(),  q.w(),  -q.x(),
                  -q.z(), -q.y(),  q.x(),   q.w();
            Er = 2 * Er;
        }

        void computeErFromYPR(const Eigen::Vector3d& ypr, Eigen::MatrixXd& Er)
        {
            double sin_z = sin(ypr[0]);
            double cos_z = cos(ypr[0]);
            double sin_y = sin(ypr[1]);
            double cos_y = cos(ypr[1]);
            
            Er.resize(3, 3);
            Er << 0, -sin_z,  cos_y * cos_z,
                  0,  cos_z,  cos_y * sin_z,
                  1,  0,     -sin_y;
            
        }
        void computeErFromRPY(const Eigen::Vector3d& rpy, Eigen::MatrixXd& Er)
        {
            double sin_x = sin(rpy[0]);
            double cos_x = cos(rpy[0]);
            double sin_y = sin(rpy[1]);
            double cos_y = cos(rpy[1]);
            
            Er.resize(3, 3);
            Er << 1, 0,      sin_y,
                  0, cos_x, -cos_y * sin_x,
                  0, sin_x,  cos_x * cos_y;
        }

        void calculateInverseTransformationMatrix(const Eigen::Matrix4d& src, Eigen::Matrix4d& dst)
        {
            dst = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d R_trans = src.block(0, 0, 3, 3).transpose();
            dst.block(0, 0, 3, 3) = R_trans;
            dst.block(0, 3, 3, 1) = -R_trans * src.block(0, 3, 3, 1);
        }

        void yprToRotationMatrix(const std::vector<double>& ypr, Eigen::Matrix3d& mat)
        {
            double sin_z = sin(ypr[0]);
            double cos_z = cos(ypr[0]);
            double sin_y = sin(ypr[1]);
            double cos_y = cos(ypr[1]);
            double sin_x = sin(ypr[2]);
            double cos_x = cos(ypr[2]);

            mat.coeffRef(0, 0) = cos_y * cos_z;
            mat.coeffRef(0, 1) = cos_z * sin_x * sin_y - cos_x * sin_z;
            mat.coeffRef(0, 2) = sin_x * sin_z + cos_x * cos_z * sin_y;
            mat.coeffRef(1, 0) = cos_y * sin_z;
            mat.coeffRef(1, 1) = cos_x * cos_z + sin_x * sin_y * sin_z;
            mat.coeffRef(1, 2) = cos_x * sin_y * sin_z - cos_z * sin_x;
            mat.coeffRef(2, 0) = -sin_y;
            mat.coeffRef(2, 1) = cos_y * sin_x;
            mat.coeffRef(2, 2) = cos_x * cos_y;

        }

        void yprToRotationMatrix(const Eigen::Vector3d& ypr, Eigen::Matrix3d& mat)
        {
            std::vector<double> ypr_vec;
            for (uint32_t i = 0; i < 3; i++)
            {
                ypr_vec.push_back(ypr.coeff(i));
            }

            yprToRotationMatrix(ypr_vec, mat);
        }

        void rpyToRotationMatrix(const std::vector<double>& rpy, Eigen::Matrix3d& mat) 
        {
            double sin_z = sin(rpy[2]);
            double cos_z = cos(rpy[2]);
            double sin_y = sin(rpy[1]);
            double cos_y = cos(rpy[1]);
            double sin_x = sin(rpy[0]);
            double cos_x = cos(rpy[0]);             

            mat.coeffRef(0, 0) = cos_y * cos_z;
            mat.coeffRef(0, 1) = -cos_y * sin_z;
            mat.coeffRef(0, 2) = sin_y;
            mat.coeffRef(1, 0) = cos_x * sin_z + cos_z * sin_x * sin_y;
            mat.coeffRef(1, 1) = cos_x * cos_z - sin_x * sin_y * sin_z;
            mat.coeffRef(1, 2) = -cos_y * sin_x;
            mat.coeffRef(2, 0) = sin_x * sin_z - cos_x * cos_z * sin_y;
            mat.coeffRef(2, 1) = cos_z * sin_x + cos_x * sin_y * sin_z;
            mat.coeffRef(2, 2) = cos_x * cos_y;            
        }

        void rpyToRotationMatrix(const Eigen::Vector3d& rpy, Eigen::Matrix3d& mat)
        {
            std::vector<double> rpy_vec;
            for (uint32_t i = 0; i < 3; i++)
            {
                rpy_vec.push_back(rpy.coeff(i));
            }

            rpyToRotationMatrix(rpy_vec, mat); 
        }

        void yprToQuaternion(const Eigen::Vector3d& ypr, Eigen::Quaternion<double>& q)
        {
            double sin_z_half = sin(ypr.coeff(0) * 0.5);
            double cos_z_half = cos(ypr.coeff(0) * 0.5);
            double sin_y_half = sin(ypr.coeff(1) * 0.5);
            double cos_y_half = cos(ypr.coeff(1) * 0.5);
            double sin_x_half = sin(ypr.coeff(2) * 0.5);
            double cos_x_half = cos(ypr.coeff(2) * 0.5);

            q.w() = cos_x_half * cos_y_half * cos_z_half 
                    + sin_x_half * sin_y_half * sin_z_half;
            q.x() = sin_x_half * cos_y_half * cos_z_half
                    - cos_x_half * sin_y_half * sin_z_half;
            q.y() = cos_x_half * sin_y_half * cos_z_half
                    + sin_x_half * cos_y_half * sin_z_half;
            q.z() = cos_x_half * cos_y_half * sin_z_half
                    - sin_x_half * sin_y_half * cos_z_half;
        }

        void rpyToQuaternion(const Eigen::Vector3d& rpy, Eigen::Quaternion<double>& q)
        {
            double sin_z_half = sin(rpy.coeff(2) * 0.5);
            double cos_z_half = cos(rpy.coeff(2) * 0.5);
            double sin_y_half = sin(rpy.coeff(1) * 0.5);
            double cos_y_half = cos(rpy.coeff(1) * 0.5);
            double sin_x_half = sin(rpy.coeff(0) * 0.5);
            double cos_x_half = cos(rpy.coeff(0) * 0.5);
            
            q.w() = cos_x_half * cos_y_half * cos_z_half 
                    - sin_x_half * sin_y_half * sin_z_half;
            q.x() = sin_x_half * cos_y_half * cos_z_half
                    + cos_x_half * sin_y_half * sin_z_half;
            q.y() = cos_x_half * sin_y_half * cos_z_half
                    - sin_x_half * cos_y_half * sin_z_half;
            q.z() = cos_x_half * cos_y_half * sin_z_half
                    + sin_x_half * sin_y_half * cos_z_half;
            
        }

        void xyzyprToTransformationMatrix(const Eigen::Vector3d& xyz, const Eigen::Vector3d& ypr, Eigen::Matrix4d& T)
        {
            T = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d R;
            yprToRotationMatrix(ypr, R);
            T.block(0, 0, 3, 3) = R;
            T.block(0, 3, 3, 1) = xyz;
        }

        void xyzrpyToTransformationMatrix(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy, Eigen::Matrix4d& T)
        {
            T = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d R;
            rpyToRotationMatrix(rpy, R);
            T.block(0, 0, 3, 3) = R;
            T.block(0, 3, 3, 1) = xyz;
        }
        
    }
}

