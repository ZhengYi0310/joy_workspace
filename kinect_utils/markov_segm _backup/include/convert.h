/*************************************************************************
	> File Name: convert.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 18 Apr 2017 01:31:53 PM PDT
 ************************************************************************/

#ifndef _CONVERT_H
#define _CONVERT_H

#include <opencv2/opencv.hpp>
#include <config.h>

namespace markov_segm
{
    void convert(cv::Mat& prob, cv::Mat& output);


}
#endif
