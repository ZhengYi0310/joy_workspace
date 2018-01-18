#ifndef __OPS_WBC_WRAPPERS_MATERIAL_HPP
#define __OPS_WBC_WRAPPERS_MATERIAL_HPP

#include <boost/shared_ptr.hpp>
#include <GL/gl.h>

namespace ops_wbc_wrappers
{
    class Material
    {
        public:
            Material(GLfloat* amb, GLfloat* dif, GLfloat* spe, GLfloat shine);
            static void setMaterial(GLfloat* amb, GLfloat* dif, GLfloat* spe, GLfloat shine);
            void apply();

        private:
            GLfloat ambient_[4];
            GLfloat diffuse_[4];
            GLfloat specular_[4];
            GLfloat shine_;
    };
    typedef boost::shared_ptr<Material> MaterialPtr;
}
#endif // __OPS_WBC_WRAPPERS_MATERIAL_HPP
