#ifndef __OPS_WBC_WRAPPERS_QUATERNION_HPP
#define __OPS_WBC_WRAPPERS_QUATERNION_HPP

#include <Eigen/Dense>
#include <ops_wbc_wrappers/exceptions.hpp>

namespace ops_wbc_wrappers
{
    class Quaternion
    {
        public:
            Quaternion();
            Quaternion(double x, double y, double z, double w);
            Quaternion(const Eigen::Vector3d& axis, double angle);
            Quaternion(const Eigen::Vector3d& pos);

            Quaternion& operator=(const Quaternion& q);
            void operator*=(Quaternion& q);
            const Quaternion operator*(Quaternion& q) const;

            Quaternion inverse();
            Quaternion& rotateWith(Quaternion& q);

            const double getNorm() const 
            {
                return sqrt(x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_);
            }

            const double getX() const 
            {
                return x_;
            }

            const double getY() const 
            {
                return y_;
            }

            const double getZ() const 
            {
                return z_;
            }

            const double getW() const 
            {
                return w_;
            }

        private:
            double x_;
            double y_;
            double z_;
            double w_;
    };
}

#endif // __OPS_WBC_WRAPPERS_QUATERNION_HPP
