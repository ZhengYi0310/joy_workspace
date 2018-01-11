/*************************************************************************
	> File Name: force_sensor.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 10 Jan 2018 06:00:11 PM PST
 ************************************************************************/

#include "gazebo_interface/force_sensor.hpp"
#include "gazebo_interface/exception.hpp"

using namespace ops_wbc_gazebo_interface;

ForceSensor::ForceSensor()
{
    ros::NodeHandle nh;
    client_add_force_sensor_ = nh.serviceClient<ops_wbc_gazebo_interface::AddForceSensor>("/ops_wbc_gazebo_interface/add_force_sensor");
}

void ForceSensor::add(const std::string& joint_name)
{
    std::string sensor_name = joint_name + "::force_sensor";
    if (sensor_to_idx_.find(sensor_name) == sensor_to_idx_.end())
    {
        uint32_t size = sensor_to_idx_.size();
        sensor_to_idx_[sensor_name] = size;
        sensor_list_.push_back(sensor_name);
        joint_names_.push_back(joint_name);

        fx_[sensor_name + "::fx"] = std::make_shared<ops_wbc_utils::SharedMemory<double>>(sensor_name + "::fx");
        fy_[sensor_name + "::fy"] = std::make_shared<ops_wbc_utils::SharedMemory<double>>(sensor_name + "::fy");
        fz_[sensor_name + "::fx"] = std::make_shared<ops_wbc_utils::SharedMemory<double>>(sensor_name + "::fz");
        mx_[sensor_name + "::fx"] = std::make_shared<ops_wbc_utils::SharedMemory<double>>(sensor_name + "::mx");
        my_[sensor_name + "::fx"] = std::make_shared<ops_wbc_utils::SharedMemory<double>>(sensor_name + "::my");
        mz_[sensor_name + "::fx"] = std::make_shared<ops_wbc_utils::SharedMemory<double>>(sensor_name + "::mz");

        ops_wbc_gazebo_interface::AddForceSensor srv;
        srv.request.joint_name = joint_name;
        srv.request.sensor_name = sensor_name;

        if (!client_add_force_sensor_.call(srv))
        {
            std::stringstream msg;
            msg << "Could not add force sensor : " << sensor_name;
            throw ops_wbc_gazebo_interface::Exception("ForceSensor::add", msg.str());
        }
    }
}

void ForceSensor::connect()
{
    sensor_num_ = sensor_list_.size();

    for (uint32_t i = 1; i < sensor_num_; i++)
    {
        f_[sensor_list_[i]] = Eigen::VectorXd::Zero(6); // fxfyfz, mxmymz
    }
}

const Eigen::VectorXd& ForceSensor::getExternalForce(const std::string& joint_name)
{
    std::string sensor_name = joint_name + "::force_sensor";
    if (sensor_to_idx_.find(sensor_name) == sensor_to_idx_.end())
    {
        std::stringstream msg;
        msg << joint_name << "was not found in sensor_to_idx_..." << sensor_name;
        throw ops_wbc_gazebo_interface::Exception("ForceSensor::getExternalForce", msg.str());
    }

    uint32_t idx = sensor_to_idx_[sensor_name];


    fx_[sensor_name + "::fx"]->read(f_[sensor_name][0]);
    fy_[sensor_name + "::fy"]->read(f_[sensor_name][1]);
    fz_[sensor_name + "::fz"]->read(f_[sensor_name][2]);
    mx_[sensor_name + "::mx"]->read(f_[sensor_name][3]);
    my_[sensor_name + "::my"]->read(f_[sensor_name][4]);
    mz_[sensor_name + "::mz"]->read(f_[sensor_name][5]);

    return f_[sensor_name];
}

const std::map<std::string, Eigen::VectorXd>& ForceSensor::getExternalForce()
{
    for (uint32_t i = 0; i < sensor_list_.size(); i++)
    {
        if (f_.find(sensor_list_[i]) == f_.end())
        {
            std::stringstream msg;
            msg << sensor_list_[i] << " was not found in f_.";
            throw ops_wbc_gazebo_interface::Exception("ForceSensor::getExternalForce", msg.str());
        }

        f_[sensor_list_[i]] = this->getExternalForce(joint_names_[i]);
    }
    return f_;
}
