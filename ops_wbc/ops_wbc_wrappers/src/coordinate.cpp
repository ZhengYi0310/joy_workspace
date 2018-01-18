/*************************************************************************
	> File Name: coordinate.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 18 Jan 2018 01:39:16 PM PST
 ************************************************************************/

#include <GL/glut.h>
#include "ops_wbc_wrappers/coordinate.hpp"

using namespace ops_wbc_wrappers;

Coordinate::Coordinate(double scale, double length) : scale_(scale), length_(length)
{

}

void Coordinate::displayImpl()
{
    glScaled(scale_, scale_, scale_);
    glDisable(GL_LIGHTING);

    const double radius = 0.03;

    glColor3d(1.0, 0.0, 0.0);
    glRotated(90.0, 0.0, 1.0, 0.0);
    glutSolidCylinder(radius, length_, 16, 16);

    glColor3d(0.0, 1.0, 0.0);
    glRotated(-90.0, 0.0, 1.0, 0.0);
    glRotated(-90.0, 1.0, 0.0, 0.0);
    glutSolidCylinder(radius, length_, 16, 16);

    glColor3d(0.0, 0.0, 1.0);
    glRotated(90.0, 1.0, 0.0, 0.0);
    glTranslated(0.0, 0.0, -radius);
    glutSolidCylinder(radius, length_ + radius, 16, 16);

    glEnable(GL_LIGHTING);
    double scale_inv = 1.0 / scale_;
    glScaled(scale_inv, scale_inv, scale_inv);
}

