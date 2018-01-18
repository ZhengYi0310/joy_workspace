/*************************************************************************
	> File Name: grid.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 18 Jan 2018 01:49:39 PM PST
 ************************************************************************/

#include "ops_wbc_wrappers/grid.hpp"
#include <GL/glut.h>

using namespace ops_wbc_wrappers;

Grid::Grid(unsigned int grid_num_x, unsigned int grid_num_y, double resolution) : grid_num_x_(grid_num_x), grid_num_y_(grid_num_y), resolution_(resolution)
{
}

void Grid::displayImpl()
{
    double start_x = 0.0;
    double start_y = 0.0;

    if(grid_num_x_ % 2 == 0)
    {
        start_x = -(grid_num_x_ / 2.0 * resolution_ - resolution_ * 0.5);
    }
  
    else
    {
        start_x = -(grid_num_x_ / 2.0 * resolution_);
    }

    if(grid_num_y_ % 2 == 0)
    {
        start_y = -(grid_num_y_ / 2.0 * resolution_ - resolution_ * 0.5);
    }
  
    else
    {
        start_y = -(grid_num_y_ / 2.0 * resolution_);
    }

    double length_x = resolution_ * (grid_num_x_ - 1);
    double length_y = resolution_ * (grid_num_y_ - 1);

    double x = start_x;
    double y = start_y;
    
    glDisable(GL_LIGHTING);
    for(unsigned int i = 0; i < grid_num_x_; ++i)
    {
        glBegin(GL_LINES);
        glVertex2d(x, y);
        glVertex2d(x, y + length_y);
        glEnd();

        x += resolution_;
    }

    x = start_x;
    for(unsigned int i = 0; i < grid_num_y_; ++i)
    {
        glBegin(GL_LINES);
        glVertex2d(x, y);
        glVertex2d(x + length_x, y);
        glEnd();

        y += resolution_;
    }
    glEnable(GL_LIGHTING);

}

