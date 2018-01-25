/*************************************************************************
	> File Name: display.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 24 Jan 2018 03:10:22 PM PST
 ************************************************************************/

#include <GL/gl.h>
#include <GL/glut.h>
#include <ops_wbc_wrappers/display.hpp>

using namespace ops_wbc_wrappers;

Display::Display(const std::string& name, int h, int w, const std::vector<int>& color)
  : name_(name), h_(h), w_(w), color_(color)
{
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(w_, h_);
    glutCreateWindow(name_.c_str());
    glClearColor(color_[0]/255.f, color_[1]/255.f, color_[2]/255.f, 1.f);
    glClearDepth(1.f);

    glEnable(GL_DEPTH_TEST);
}

