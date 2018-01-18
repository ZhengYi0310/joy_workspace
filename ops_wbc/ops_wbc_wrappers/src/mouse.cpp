/*************************************************************************
	> File Name: mouse.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 17 Jan 2018 08:36:40 PM PST
 ************************************************************************/

#include <GL/gl.h>
#include <GL/glut.h>
#include "ops_wbc_wrappers/mouse.hpp"

using namespace ops_wbc_wrappers;

void Mouse::click(int button, int state, int x, int y)
{
    button_ = button;
    state_ = state;
    pre_x_ = x;
    pre_y_ = y;
    dx_ = 0;
    dy_ = 0;
}

void Mouse::drag(int x, int y)
{
    dx_ = x - pre_x_;
    dy_ = y - pre_y_;
    pre_x_ = x;
    pre_y_ = y;
}

void Mouse::scroll(int wheel_id, int direction, int x, int y)
{}

