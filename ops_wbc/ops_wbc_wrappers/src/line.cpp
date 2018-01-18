/*************************************************************************
	> File Name: line.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 18 Jan 2018 01:44:40 PM PST
 ************************************************************************/

#include "ops_wbc_wrappers/line.hpp"

using namespace ops_wbc_wrappers;

Line::Line(unsigned int max_size)
  : max_size_(max_size)
{

}

void Line::push_back(double x, double y, double z)
{
  x_.push_back(x);
  y_.push_back(y);
  z_.push_back(z);
}

void Line::display()
{

}

