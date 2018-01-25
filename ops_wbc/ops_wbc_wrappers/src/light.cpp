/*************************************************************************
	> File Name: light.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 24 Jan 2018 03:42:02 PM PST
 ************************************************************************/

#include <ops_wbc_wrappers/light.hpp>

using namespace ops_wbc_wrappers;

int Light::light_id = 0;

Light::Light(const Eigen::Vector4d& pos, const Eigen::Vector4d& amb, const Eigen::Vector4d& dif, const Eigen::Vector4d& spe)
{
    for (int i = 0; i < 4; i++)
    {
        pos_[i]      = pos.coeff(i);
        ambient_[i]  = amb.coeff(i);
        diffuse_[i]  = dif.coeff(i);
        specular_[i] = spe.coeff(i);
    }

    ++Light::light_id;
}

void Light::on()
{
    glLightfv(GL_LIGHT0 + Light::light_id, GL_AMBIENT, ambient_);
    glLightfv(GL_LIGHT0 + Light::light_id, GL_DIFFUSE, diffuse_);
    glLightfv(GL_LIGHT0 + Light::light_id, GL_SPECULAR, specular_);
    glLightfv(GL_LIGHT0 + Light::light_id, GL_POSITION, pos_);
    glEnable(GL_LIGHT0 + Light::light_id);
    glEnable(GL_LIGHTING);
}

void Light::off()
{
    glDisable(GL_LIGHT0 + Light::light_id);
    glDisable(GL_LIGHTING);
}

