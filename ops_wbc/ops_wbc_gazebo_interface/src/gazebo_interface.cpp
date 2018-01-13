/*************************************************************************
	> File Name: gazebo_interface.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 10 Jan 2018 10:27:13 PM PST
 ************************************************************************/

#include "gazebo_interface/gazebo_interface.hpp"
#include "gazebo_interface/exception.hpp"

using namespace ops_wbc_gazebo_interface

GazeboInterface::GazeboInterface() : subscribed_joint_states_(true)
{
    torque_sensor_ = std::make_shared<TorqueSensor>();
    force_sensor_ = std::make_shared<ForceSensor>();

    ros::NodeHandle nh;
    client_start_timer_ = nh.serviceClient<gazebo_msgs::S
}

