/*************************************************************************
	> File Name: x_left_hand.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 29 Jan 2018 03:32:25 PM PST
 ************************************************************************/

#include<iostream>
#include <ops_wbc_wrappers/x_left_hand.hpp>
using namespace ops_wbc_wrappers;

XLeftHand::XLeftHand(const std::string& config_name, double scale)
            : config_name_(config_name), scale_(scale)
{
}

void XLeftHand::display()
{
    glScaled(scale_, scale_, scale_);
    this->getLeftHand()->display();
    glScaled(1.0 / scale_, 1.0 / scale_, 1.0 / scale_);
}

LeftHandPtr& XLeftHand::getLeftHand()
{
  static LeftHandPtr left_hand = LeftHandPtr(new LeftHand(config_name_));
  return left_hand;
}

