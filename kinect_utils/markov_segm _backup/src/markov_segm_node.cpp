/*************************************************************************
	> File Name: markov_segm_node.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 19 Apr 2017 02:02:11 PM PDT
 ************************************************************************/

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <cassert>

#include <ros/ros.h>
#include <simulated_annealing.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "markov_segmentation_code");
    ros::NodeHandle nh("markov_segm");

    markov_segm::SimulatedAnnealing simulated_annealing(nh);
    ros::spin();

    return 0;

}

