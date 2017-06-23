/*************************************************************************
	> File Name: config.h
    > Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 18 Apr 2017 12:23:09 PM PDT
 ************************************************************************/

#ifndef _CONFIG_H
#define _CONFIG_H

#include <opencv2/opencv.hpp>

namespace markov_segm
{
    #define NB_COLORS 3

    class Config 
    {
        public:
            static cv::Vec3b colors[NB_COLORS];
            static double beta;
            static double initial_temperature;
            static double temperature_decrease;
            static double min_change;
    };

}
#endif
