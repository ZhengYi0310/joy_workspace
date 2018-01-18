/*************************************************************************
	> File Name: Object.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 17 Jan 2018 10:33:04 PM PST
 ************************************************************************/

#include <ops_wbc_wrappers/object.hpp>
#include <ops_wbc_wrappers/exceptions.hpp>

using namespace ops_wbc_wrappers;

Object::Object()
  : assign_normal_to_vertex_(true), assign_material_to_vertex_(true)
{

}

void Object::display()
{
    if(this->haveEmptyParam())
    {
        throw Exception("Object::display", "Could not display because the object has empty params.");
    }

    glTranslated(offset.translation.coeff(0),
                offset.translation.coeff(1),
                offset.translation.coeff(2));

    glRotated(offset.euler_zyx.coeff(0) * 180.0 / M_PI, 0.0, 0.0, 1.0);
    glRotated(offset.euler_zyx.coeff(1) * 180.0 / M_PI, 0.0, 1.0, 0.0);
    glRotated(offset.euler_zyx.coeff(2) * 180.0 / M_PI, 1.0, 0.0, 0.0);

    glTranslated(motion.translation.coeff(0),
                motion.translation.coeff(1),
                motion.translation.coeff(2));

    glRotated(motion.euler_zyx.coeff(0) * 180.0 / M_PI, 0.0, 0.0, 1.0);
    glRotated(motion.euler_zyx.coeff(1) * 180.0 / M_PI, 0.0, 1.0, 0.0);
    glRotated(motion.euler_zyx.coeff(2) * 180.0 / M_PI, 1.0, 0.0, 0.0);

    glCallList(model_list);
}

bool Object::haveEmptyParam()
{
    if(vertex.size() == 0 || v_idx.size() == 0 || normal.size() == 0 || n_idx.size() == 0 || material.size() == 0 || material_assignment.size() == 0)
    {
        return true;
    }

    return false;
}

