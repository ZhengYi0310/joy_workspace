/*************************************************************************
	> File Name: material.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 17 Jan 2018 10:16:51 PM PST
 ************************************************************************/

#include <ops_wbc_wrappers/material.hpp>

using namespace ops_wbc_wrappers;

Material::Material(GLfloat* amb, GLfloat* dif, GLfloat* spe, GLfloat shine)
{
    for(int i = 0; i < 4; ++i)
    {
        ambient_[i]  = amb[i];
        diffuse_[i]  = dif[i];
        specular_[i] = spe[i];
        shine_       = shine;
    }
}

void Material::setMaterial(GLfloat* amb, GLfloat* dif, GLfloat* spe, GLfloat shine)
{
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   dif);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  spe);
    glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, shine);
}

void Material::apply()
{
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   ambient_);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   diffuse_);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  specular_);
    glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, shine_);
}

