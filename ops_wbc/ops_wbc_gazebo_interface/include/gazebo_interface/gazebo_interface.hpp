#ifndef __OPS_WBC_GAZEBO_INTERFACE_GAZEBO_INTERFACE_HPP
#define __OPS_WBC_GAZEBO_INTERFACE_GAZEBO_INTERFACE_HPP

#include <map>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ops_wbc_gazebo_interface/AddJoint.h>
#include <gazebo_msgs/StartTimer.h>
#include <gazebo_msgs/LinkStates.h>
#include <ops_wbc_utils/shared_memory.hpp>
#include <ops_wbc_utils/scoped_lock.hpp>
#include "ops_wbc_gazebo_interface/torque_sensor.hpp"
#include "ops_wbc_gazebo_interface/force_sensor.hpp"

namespace ops_wbc_gazebo_interface
{
    // static constant value which defines the topic name for link states
    static const std::string TOPIC_SUB_LINK_STATES = "/gazebo/set_link_states";

    // This class enables communication with gazebo simulator through ops_wbc_utils::shared_memory. 
    class GazeboInterface
    {
        public:
            explicit GazeboInterface();
            ~GazeboInterface();

            // Register joint name to get joint angle or apply torque 
            //@param name Name of the joint you'd like to use
            //@param effort_time Defines how long you'd like to apply the effort to the joint when calling applyJointEfforts.
            void addJoint(const std::string& name, double effort_time = 0.010);

            // Register torque sensor to a specified joint 
            // Joint of macro manipulator shouldn't have torque sensor 
            // @param name Name of the joint you'd like to put a torque sensor on
            void addTorqueSensor(const std::string& joint_name);

            // Register force sensor to a specified joint 
            // Sensor will be attached to the position of the specified joint 
            // @param name Name of the joint you'd like to put a force sensor on
            void addForceSensor(const std::string& joint_name);

            // Initialize and connect the communication with gazebo simulator 
            void connect();

            // Apply torque to registered joint 
            // @param tau Toruqe vector. 
            void applyJointEfforts(const Eigen::VectorXd& tau);

            // Register link to translate or rotate without considering dynamics 
            // @param robot Name of the robot.
            // @param link Name of the link to register 
            void addLink(const std::string& robot, const std::string& link);

            // Translate link 
            // @param p xyz position vector w.r.t parent link frame 
            void translateLink(const std::vector<Eigen::Vector3d>& p);

            // Rotate link
            // @param q Quarternion w.r.t parent link frame 
            void rotateLink(const std::vector<Eigen::Quarternion<double> >& q);

            // Get joint angles
            // @return Joint state vector representing joint angles or displacements
            const Eigen::VectorXd& getJointStates();

            const Eigen::VectorXd& getJointTorque();

            const Eigen::VectorXd& getExternalForce(const std::string& joint_name);

            const std::map<std::string, Eigen::VectorXd>& getExternalForce();

        private:
            std::mutex mutex_;
            
            // <joint name, joint index representing the registration order>
            std::map<std::string, int32_t> joint_to_idx_;
            // joint name list 
            std::vector<std::string> joint_list_;

            // number of registered joint 
            unsigned int joint_num_;

            // joint angle/displacement 
            Eigen::VectorXd q_;

            // True: Gazebo simulator has written some value to shared memory 
            // False: Gazebo simulator hasn't written any values to shared memory yet 
            bool subscribed_joint_states_;

            // Publisher to publich link rotation and orientation
            ros::Publisher pub_link_states_;

            // Link positions and orientations 
            gazebo_msgs::LinkStates link_states_;

            // ROS service client to call service server provided by gazebo_ros to start writing joint angles/displacements 
            ros::ServiceClient client_start_timer_;

            // ROS service client to call service server provieded by gazebo_ros to register joint 
            ros::ServiceClient client_add_joint_;

            // <joint name, torque to apply>
            std::map<std::string, ops_wbc_utils::SharedMemoryPtr<double>> joint_effort_;

            // <joint_name, joint angle/displacements>
            std::map<std::string, ops_wbc_utils::SharedMemoryPtr<double>> joint_state_;

            TorqueSensorPtr torque_sensor_;
            ForceSensor force_sensor_;
    };
    using GazeboInterfacePtr = std::shared_ptr<GazeboInterface>;
}
#endif // __OPS_WBC_GAZEBO_INTERFACE_GAZEBO_INTERFACE_HPP 

