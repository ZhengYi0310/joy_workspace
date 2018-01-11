#ifndef __OPS_WBC_GAZEBO_INTERFACE_TORQUE_SENSOR_HPP
#define __OPS_WBC_GAZEBO_INTERFACE_TORQUE_SENSOR_HPP

#include <string>
#include <map>
#include <memory>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ops_wbc_gazebo_interface/AddTorqueSensor.h>
#include <utils/shared_memory.hpp>

namespace ops_wbc_gazebo_interface
{
    class TorqueSensor
    {
        public:
            explicit TorqueSensor();

            void add(const std::string& joint_name);
            void connect();

            const Eigen::VectorXd& getJointTorque();

        private:
            std::map<std::string, int32_t> sensor_to_idx_;
            std::vector<std::string> sensor_list_;
            std::vector<std::string> joint_names_;
            uint32_t sensor_num_;
            Eigen::VectorXd tau_;
            std::map<std::string, ops_wbc_utils::SharedMemoryPtr<double>> torque_;
            ros::ServiceClient client_add_torque_sensor_;
    };
    using TorqueSensorPtr = std::shared_ptr<TorqueSensor>;
}

#endif // __OPS_WBC_GAZEBO_INTERFACE_TORQUE_SENSOR_HPP 
