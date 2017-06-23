/*************************************************************************
	> File Name: random.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 18 Apr 2017 01:38:01 PM PDT
 ************************************************************************/

#ifndef _RANDOM_H
#define _RANDOM_H

#include <random>
#include <opencv2/opencv.hpp>
#include <config.h>

namespace markov_segm
{
    void random_image(cv::Mat& img);
}
#endif
