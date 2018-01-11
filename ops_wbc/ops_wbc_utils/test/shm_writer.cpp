/*************************************************************************
	> File Name: shm_writer.cpp
	> Author: Yi Zheng 
    > Mail: hczhengcq@gmail.com
	> Created Time: Wed 10 Jan 2018 04:16:55 PM PST
 ************************************************************************/

#include<iostream>
#include<ros/ros.h>
#include "utils/shared_memory.hpp"
using namespace std;
using namespace ops_wbc_utils;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "shm_writer");
    ros::NodeHandle nh;

    SharedMemoryPtr<double> shm = SharedMemoryPtr<double>(new SharedMemory<double>("shm"));

    ros::Rate r(1000.0);
    double data = 0;

    while(ros::ok())
    {
        shm->write(data);
        data += 0.001;
        std::cout << data << std::endl;
        r.sleep();
    }

    return 0;
}

