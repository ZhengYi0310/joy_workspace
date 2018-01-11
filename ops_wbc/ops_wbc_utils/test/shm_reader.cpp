/*************************************************************************
	> File Name: shm_reader.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 10 Jan 2018 04:05:54 PM PST
 ************************************************************************/

#include <ros/ros.h>
#include "utils/shared_memory.hpp"
using namespace ops_wbc_utils;

int main(int argc, char** argv)
{
    ros::init(argc,argv, "shm_reader");
    ros::NodeHandle nh;

    SharedMemoryPtr<double> shm = SharedMemoryPtr<double>(new SharedMemory<double>("shm"));

    ros::Rate r(1000.0);

    double data = 0;
    while(ros::ok())
    {
        shm->read(data);
        std::cout << data << std::endl;
        r.sleep();
    }

    return 0;
}

