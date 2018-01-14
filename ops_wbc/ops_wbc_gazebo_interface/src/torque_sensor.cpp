/*************************************************************************
	> File Name: torque_sensor.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 10 Jan 2018 09:19:26 PM PST
 ************************************************************************/

#include "gazebo_interface/torque_sensor.hpp"
#include "gazebo_interface/exception.hpp"

using namespace ops_wbc_gazebo_interface;

TorqueSensor::TorqueSensor()
{
    ros::NodeHandle nh;
    client_add_torque_sensor_ = nh.serviceClient<gazebo_msgs::AddTorqueSensor>("/gazebo/add_torque_sensor");
}

void TorqueSensor::add(const std::string& joint_name)
{
    std::string sensor_name = joint_name + "::torque_sensor";
    if (sensor_to_idx_.find(sensor_name) == sensor_to_idx_.end())
    {
        uint32_t size = sensor_to_idx_.size();
        sensor_to_idx_[sensor_name] = size;
        sensor_list_.push_back(sensor_name);
        joint_names_.push_back(joint_name);

        torque_[sensor_name] = std::make_shared<ops_wbc_utils::SharedMemory<double>>(sensor_name);

        gazebo_msgs::AddTorqueSensor srv;
        srv.request.joint_name = joint_name;
        srv.request.sensor_name = sensor_name;

        if (!client_add_torque_sensor_.call(srv))
        {
            std::stringstream msg;
            msg << "Could not add torque sensor : " << sensor_name;
            throw ops_wbc_gazebo_interface::Exception("TorqueSensor::add", msg.str());
        }
    }
}

void TorqueSensor::connect()
{
    sensor_num_ = sensor_list_.size();
    tau_ = Eigen::VectorXd::Zero(sensor_num_);
}

const Eigen::VectorXd& TorqueSensor::getJointTorque()
{
    for (uint32_t i = 0; i < sensor_num_; i++)
    {
        if (sensor_to_idx_.find(sensor_list_[i]) == sensor_to_idx_.end())
        {
            std::stringstream msg;
            msg << sensor_list_[i] << " was not found in sensor_to_idx_.";
            throw ops_wbc_gazebo_interface::Exception("TorqueSensor::getSensedTorque", msg.str());
        }

        torque_[sensor_list_[i]]->read(tau_[i]);
    }
}
