/*************************************************************************
	> File Name: hsv_mask.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 17 Jan 2018 03:26:20 PM PST
 ************************************************************************/

#include "ops_wbc_wrappers/utils.hpp"
#include "ops_wbc_wrappers/hsv_mask.hpp"
#include "ops_wbc_wrappers/exceptions.hpp"

using namespace ops_wbc_wrappers;
HSVMask::HSVMask(int h, int w) : mask_(h, w, CV_8UC1)
{}

void HSVMask::set(const HSVMaskParamPtr& param, int idx)
{
    if (idx > param_.size() - 1)
    {
        std::stringstream ss;
        ss << "idx is larger than param size." << std::endl
           << "  idx  : " << idx << std::endl
           << "  size : " << param_.size() << std::endl;
        throw Exception("HSVMask::set", ss.str());
    }
    param_[idx] = param;
}

void HSVMask::add(const HSVMaskParamPtr& param)
{
    param_.push_back(param);
}

void HSVMask::getMask(std::vector<cv::Mat>& hsv, bool labeling, int erode_num, int dilate_num)
{
    mask_.setTo(cv::Scalar(0));
    cv::Mat temp(mask_.rows, mask_.cols, CV_8UC1, cv::Scalar(0));

    for (unsigned int i = 0; i < param_.size(); i++)
    {
        extractDesignatedHSVArea(hsv, temp,
                                 param_[i]->getHMin(), param_[i]->getHMax(),
                                 param_[i]->getSMin(), param_[i]->getSMax(),
                                 param_[i]->getVMin(), param_[i]->getVMax(),
                                 labeling, erode_num, dilate_num);
        cv::bitwise_or(temp, mask_, mask_);
    }
}

const cv::Mat& HSVMask::get() const 
{
    return mask_;
}

