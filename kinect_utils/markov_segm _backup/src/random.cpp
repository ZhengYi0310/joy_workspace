/*************************************************************************
	> File Name: random.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 19 Apr 2017 02:41:32 PM PDT
 ************************************************************************/

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <cassert>
#include <random.h>
#include <config.h>
#include <random>
namespace markov_segm
{
    void random_image(cv::Mat& img)
    {
        std::mt19937 eng_(time(NULL));
        std::uniform_int_distribution<uint32_t> uint_dist_;

        // randomly initialize the class 
        for (int i = 0; i < img.rows; ++i)
        {
            for (int j = 0; j < img.cols; j++)
            {
                img.at<uchar>(i, j) = uint_dist_(eng_) % NB_COLORS;
            }
        }
    }
}

