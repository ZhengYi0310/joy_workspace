#ifndef __OPS_WBC_ROBOT_TF_PUBLISHER_HPP
#define __OPS_WBC_ROBOT_TF_PUBLISHER_HPP

#include <memory>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "ops_wbc_robot/robot/robot.hpp"

namespace ops_wbc_robot
{
    // Publish tf depending on the state of the ops_wbc_robot::Robot 
    class TfPublisher
    {
        public:
            // Constructor 
            explicit TfPublisher();

            // Publish tf 
            // &param shared pointer of the robot which you'd like to see in the frames 
            // &param publish_com if is true, published frames are attached to the center of mass of each link 
            void publish(const RobotPtr& robot, bool publish_com=true);
            private:
                // Publish tf 
                // &param shared pointer of the manipulator which you'd like to see in the frames 
                // &param publish_com if is true, published frames are attached to the center of mass of each link 
                void publish(const ManipulatorPtr& mnp, const ros::Time& current, bool publish_com=true);

                //! Singleton of transform broadcaster
                tf2_ros::TransformBroadcaster& transformBroadcaster();
    };
    using TfPublisherPtr = std::shared_ptr<TfPublisher>;
}

#endif //__OPS_WBC_ROBOT_TF_PUBLISHER_HPP
