#ifndef __OPS_WBC_ROBOT_MATH_HPP
#define __OPS_WBC_ROBOT_MATH_HPP

#include <vector>
#include <Eigen/Dense>
#include <math.h>

namespace ops_wbc_robot
{
    namespace math
    {
        /// Compute matrix which convert from quaternion to 3 dimensional vector
        /// \param q Quaternion
        /// \param Er Conversion matrix
        void computeEr(const Eigen::Quaternion<double>& q, Eigen::MatrixXd& Er);
        void computeErFromYPR(const Eigen::Vector3d& ypr, Eigen::MatrixXd& Er);
        void computeErFromRPY(const Eigen::Vector3d& ypr, Eigen::MatrixXd& Er);
        void calculateInverseTransformationMatrix(const Eigen::Matrix4d& src, Eigen::Matrix4d& dst);
        void yprToRotationMatrix(const std::vector<double>& ypr, Eigen::Matrix3d& mat);
        void rpyToRotationMatrix(const std::vector<double>& rpy, Eigen::Matrix3d& mat);
        void yprToRotationMatrix(const Eigen::Vector3d& ypr, Eigen::Matrix3d& mat);
        void rpyToRotationMatrix(const Eigen::Vector3d& rpy, Eigen::Matrix3d& mat);
        void yprToQuaternion(const Eigen::Vector3d& ypr, Eigen::Quaternion<double>& q);
        void rpyToQuaternion(const Eigen::Vector3d& rpy, Eigen::Quaternion<double>& q);
        void xyzyprToTransformationMatrix(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy, Eigen::Matrix4d& T);
        void xyzrpyToTransformationMatrix(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy, Eigen::Matrix4d& T);
    }
}
#endif
