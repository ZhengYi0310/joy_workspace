#ifndef __OPS_WBC_WRAPPERS_SIMPLE_OBJECT_HPP
#define __OPS_WBC_WRAPPERS_SIMPLE_OBJECT_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

namespace ops_wbc_wrappers
{
    class SimpleObject
    {
        public:
            SimpleObject();
            virtual ~SimpleObject() {}
            virtual void display();
            virtual void setColor(int r, int g, int b);
            virtual void setPosition(const Eigen::Vector3d& pos);
            virtual void setPosition(double x, double y, double z);
            virtual void setEulerZYX(const Eigen::Vector3d& euler_zyx);
            virtual void setEulerZYX(double rz, double ry, double rx);

        private:
            virtual void displayImpl() = 0;

            Eigen::Vector3d pos_;
            Eigen::Vector3d euler_zyx_;

            int r_;
            int g_;
            int b_;
    };

    typedef boost::shared_ptr<SimpleObject> SimpleObjectPtr;
}
#endif // __OPS_WBC_WRAPPERS_SIMPLE_OBJECT_HPP
