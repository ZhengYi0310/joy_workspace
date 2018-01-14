#ifndef __OPS_WBC_GAZEBO_INTERFACE_FORCE_SENSOR_HPP
#define __OPS_WBC_GAZEBO_INTERFACE_FORCE_SENSOR_HPP

#include <string>
#include <map>
#include <memory>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <gazebo_msgs/AddForceSensor.h>
#include <utils/shared_memory.hpp>

namespace ops_wbc_gazebo_interface
{
    class ForceSensor
    {
        public:
            explicit ForceSensor();
            
            void add(const std::string& joint_name);
            void connect();

            const Eigen::VectorXd& getExternalForce(const std::string& joint_name);
            const std::map<std::string, Eigen::VectorXd>& getExternalForce();

        private:
            std::map<std::string, uint32_t> sensor_to_idx_;
            std::vector<std::string> sensor_list_;
            std::vector<std::string> joint_names_;
            uint32_t sensor_num_;

            std::map<std::string, Eigen::VectorXd> f_;
            std::map<std::string, ops_wbc_utils::SharedMemoryPtr<double>> fx_;
            std::map<std::string, ops_wbc_utils::SharedMemoryPtr<double>> fy_;
            std::map<std::string, ops_wbc_utils::SharedMemoryPtr<double>> fz_;
            std::map<std::string, ops_wbc_utils::SharedMemoryPtr<double>> mx_;
            std::map<std::string, ops_wbc_utils::SharedMemoryPtr<double>> my_;
            std::map<std::string, ops_wbc_utils::SharedMemoryPtr<double>> mz_;
            ros::ServiceClient client_add_force_sensor_;
    };

    using ForceSensorPtr = std::shared_ptr<ForceSensor>;
}

#endif // __OPS_WBC_GAZEBO_INTERFACE_FORCE_SENSOR_HPP
