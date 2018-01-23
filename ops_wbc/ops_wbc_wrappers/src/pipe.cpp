/*************************************************************************
	> File Name: pipe.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 18 Jan 2018 08:15:22 PM PST
 ************************************************************************/

#include <iostream>
#include <ops_wbc_wrappers/pipe.hpp>

using namespace ops_wbc_wrappers;

Pipe::Pipe(double h, double r_in, double r_out, unsigned int slices, unsigned int stacks)
  : h_(h), r_in_(r_in), r_out_(r_out), slices_(slices), stacks_(stacks)
{
  model_list_ = glGenLists(1);
}

void Pipe::displayImpl()
{
  glutSolidCylinder(r_in_, h_, slices_, stacks_);
}

