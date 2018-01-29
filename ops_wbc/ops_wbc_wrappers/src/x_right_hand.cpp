/*************************************************************************
	> File Name: x_right_hand.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 29 Jan 2018 03:41:34 PM PST
 ************************************************************************/

#include <ops_wbc_wrappers/x_right_hand.hpp>
using namespace ops_wbc_wrappers;

XRightHand::XRightHand(const std::string& config_name, double scale)
  : config_name_(config_name), scale_(scale)
{
}

void XRightHand::display()
{
    glScaled(scale_, scale_, scale_);
    this->getRightHand()->display();
    glScaled(1.0 / scale_, 1.0 / scale_, 1.0 / scale_);
}

RightHandPtr& XRightHand::getRightHand()
{
    static RightHandPtr right_hand = RightHandPtr(new RightHand(config_name_));
    return right_hand;
}

