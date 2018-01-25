#ifndef __OPS_WBC_WRAPPERS_LIGHT_HPP
#define __OPS_WBC_WRAPPERS_LIGHT_HPP

#include <GL/gl.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ops_wbc_wrappers
{
    class Light
    {
        public:
            Light(const Eigen::Vector4d& pos, const Eigen::Vector4d& amb, const Eigen::Vector4d& dif, const Eigen::Vector4d& spe);
            
            void on();
            void off();

            static int light_id;

        private:
            GLfloat pos_[4];
            GLfloat ambient_[4];
            GLfloat diffuse_[4];
            GLfloat specular_[4];
    };

    typedef boost::shared_ptr<Light> LightPtr;
}

#endif // __OPS_WBC_WRAPPERS_LIGHT_HPP
