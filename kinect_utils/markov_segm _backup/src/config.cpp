/*************************************************************************
	> File Name: config.cpp
	> Author: 
	> Mail: 
	> Created Time: Wed 19 Apr 2017 01:51:31 PM PDT
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

namespace markov_segm
{
    cv::Vec3b Config::colors[NB_COLORS] =
    {
        cv::Vec3b(0, 0, 228),
        cv::Vec3b(255, 255, 255),
        cv::Vec3b(44, 224, 20),
	/*
        cv::Vec3b(238, 0, 238),
        cv::Vec3b(0, 255, 127),
        cv::Vec3b(0, 0, 255),
        cv::Vec3b(36, 127, 255),
        cv::Vec3b(51, 255, 255),
        cv::Vec3b(255, 255, 51)
	*/
    };

    double Config::beta = 1.5;
    double Config::initial_temperature = 10.0;
    double Config::temperature_decrease = 0.95;
    double Config::min_change = 2.0;
}

