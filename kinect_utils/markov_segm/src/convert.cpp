/*************************************************************************
	> File Name: convert.cpp
	> Author: 
	> Mail: 
	> Created Time: Wed 19 Apr 2017 02:38:58 PM PDT
 ************************************************************************/

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <cassert>
#include <config.h>
#include <convert.h>

namespace markov_segm
{
    void convert(cv::Mat& prob, cv::Mat& output)
    {
        for (int i = 0; i < prob.rows; i++)
        {
            for (int j = 0; j < prob.cols; j++)
            {
                output.at<cv::Vec3b>(i, j) = Config::colors[prob.at<uchar>(i, j)];
            }
        }
    }
}

