/*************************************************************************
	> File Name: tf_publisher.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Sun 04 Feb 2018 04:25:11 PM PST
 ************************************************************************/

#include <utils/exception.hpp>
#include "ops_wbc_robot/definition.hpp"
#include "ops_wbc_robot/robot/tf_publisher.hpp"

using namespace ops_wbc_robot;

TfPublisher::TfPublisher()
{
}

void TfPublisher::publish(const RobotPtr& robot, bool publish_com)
{
    ros::Time current = ros::Time::now();

    for (auto it = std::begin(robot->getManipulatorName()); it != std::end(robot->getManipulatorName()); it++)
    {
        ManipulatorPtr mnp = robot->getManipulator(*it);
        this->publish(mnp, current, publish_com);
    }
}

void TfPublisher::publish(const ManipulatorPtr& mnp, const ros::Time& current, bool publish_com)
{
    for (uint32_t i = 0; i < mnp->getLinkNum(); i++)
    {
        geometry_msgs::TransformStamped tf_stamped;
        tf_stamped.header.frame_id = mnp->getLink(i)->parent;
        tf_stamped.child_frame_id  = mnp->getLink(i)->name;
        tf_stamped.header.stamp = current;

        Eigen::Matrix3d R = mnp->getTransform(i).block(0, 0, 3, 3);
        Eigen::Quaternion<double> q(R);

        tf_stamped.transform.translation.x = mnp->getTransform(i).coeff(0, 3);
        tf_stamped.transform.translation.y = mnp->getTransform(i).coeff(1, 3);
        tf_stamped.transform.translation.z = mnp->getTransform(i).coeff(2, 3);

        tf_stamped.transform.rotation.x = q.x();
        tf_stamped.transform.rotation.y = q.y();
        tf_stamped.transform.rotation.z = q.z();
        tf_stamped.transform.rotation.w = q.w();
        transformBroadcaster().sendTransform(tf_stamped);

        if (!publish_com)
            continue;

        geometry_msgs::TransformStamped com_stamped;
        com_stamped.header.frame_id = mnp->getLink(i)->name;
        com_stamped.child_frame_id  = mnp->getLink(i)->name + "_com";
        com_stamped.header.stamp    = current;

        q = Eigen::Matrix3d::Identity();

        com_stamped.transform.translation.x = mnp->getLink(i)->C.coeff(0);
        com_stamped.transform.translation.y = mnp->getLink(i)->C.coeff(1);
        com_stamped.transform.translation.z = mnp->getLink(i)->C.coeff(2);

        com_stamped.transform.rotation.x = q.x();
        com_stamped.transform.rotation.y = q.y();
        com_stamped.transform.rotation.z = q.z();
        com_stamped.transform.rotation.w = q.w();
        transformBroadcaster().sendTransform(com_stamped);
    }
}

tf2_ros::TransformBroadcaster& TfPublisher::transformBroadcaster()
{
  static tf2_ros::TransformBroadcaster br;
  return br;
}

