/*************************************************************************
	> File Name: gazebo_interface.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 10 Jan 2018 10:27:13 PM PST
 ************************************************************************/

#include "gazebo_interface/gazebo_interface.hpp"
#include "gazebo_interface/exception.hpp"

using namespace ops_wbc_gazebo_interface;

GazeboInterface::GazeboInterface() : subscribed_joint_states_(true)
{
    torque_sensor_ = std::make_shared<TorqueSensor>();
    force_sensor_ = std::make_shared<ForceSensor>();

    ros::NodeHandle nh;
    client_start_timer_ = nh.serviceClient<gazebo_msgs::StartTimer>("/gazebo/start_timer");
    client_add_joint_   = nh.serviceClient<gazebo_msgs::AddJoint>("/gazebo/add_joint");

    client_start_timer_.waitForExistence(ros::Duration(1.0));
    client_add_joint_.waitForExistence(ros::Duration(1.0));
}

GazeboInterface::~GazeboInterface()
{
    this->applyJointEfforts(Eigen::VectorXd::Zero(joint_list_.size()));
}

void GazeboInterface::addJoint(const std::string& name, double effort_time)
{
    if (joint_to_idx_.find(name) == joint_to_idx_.end())
    {
        uint32_t size = joint_to_idx_.size();
        joint_to_idx_[name] = size;
        joint_list_.push_back(name);

        joint_effort_[name] = std::make_shared<ops_wbc_utils::SharedMemory<double>>(name + "::effort");
        joint_state_[name] = std::make_shared<ops_wbc_utils::SharedMemory<double>>(name + "::state");

        gazebo_msgs::AddJoint srv;
        srv.request.name = name;
        srv.request.effort_time = effort_time;
        if (!client_add_joint_.call(srv))
        {
            std::stringstream msg;
            msg << "Could not add joint : " << name << std::endl
                << "Launch simulator first.";
            throw ops_wbc_gazebo_interface::Exception("GazeboInterface::addJoint", msg.str());
        }
    }
}

void GazeboInterface::addTorqueSensor(const std::string& joint_name)
{
    torque_sensor_->add(joint_name);
}

void GazeboInterface::addForceSensor(const std::string& joint_name)
{
    force_sensor_->add(joint_name);
}

void GazeboInterface::connect()
{
    joint_num_ = joint_list_.size();
    q_ = Eigen::VectorXd::Zero(joint_num_);

    this->force_sensor_->connect();
    this->torque_sensor_->connect();

    gazebo_msgs::StartTimer srv;
    if (!client_start_timer_.call(srv))
    {
        throw ops_wbc_gazebo_interface::Exception("GazeboInterface::connect", "Could not start timer.");
    }

    ros::NodeHandle nh;
    pub_link_states_ = nh.advertise<gazebo_msgs::LinkStates>("/gazebo/set_link_states", 1);
}

bool GazeboInterface::subscribed()
{
    ops_wbc_utils::ScopedLock lock(mutex_);
    return subscribed_joint_states_;
}

void GazeboInterface::applyJointEfforts(const Eigen::VectorXd& tau)
{
    if (tau.rows() != joint_list_.size())
    {
        std::stringstream msg;
        msg << "tau.rows() != joint_list_.size() + joint_idx_offset_" << std::endl
        << "  tau.rows()        : " << tau.rows() << std::endl
        << "  joint_list_.size() : " << joint_list_.size();

        throw ops_wbc_gazebo_interface::Exception("ops_wbc_gazebo_interface::GazeboInterface::applyJointEfforts", msg.str());
    }

    for (uint32_t i = 0; i < joint_list_.size(); i++)
    {
        if (joint_to_idx_.find(joint_list_[i]) == joint_to_idx_.end())
        {
            std::stringstream msg;
            msg << joint_list_[i] << " was not found in joint_to_idx_.";
            throw ops_wbc_gazebo_interface::Exception("ops_wbc_gazebo_interface::GazeboInterface::applyJointEffots", msg.str());
        }

        joint_effort_[joint_list_[i]]->write(tau[i]);
    }
}

void GazeboInterface::addLink(const std::string& robot, const std::string& link)
{
    std::string name = robot + "::" + link;

    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;

    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    link_states_.name.push_back(name);
    link_states_.pose.push_back(pose);
    link_states_.twist.push_back(twist);
    link_states_.reference_frame.push_back(link);
}

void GazeboInterface::translateLink(const std::vector<Eigen::Vector3d>& p)
{
    if (p.size() != link_states_.name.size())
    {
        std::stringstream msg;
        msg << "p.size() != link_states_.name.size()" << std::endl 
        << "  p.size : " << p.size() << std::endl
        << "  link_states_.name.size : " << link_states_.name.size();
        throw ops_wbc_gazebo_interface::Exception("GazeboInterface::translateLink", msg.str());
    }

    for (uint32_t i = 0; i < p.size(); i++)
    {
        link_states_.pose[i].position.x = p[i].coeff(0);
        link_states_.pose[i].position.y = p[i].coeff(1);
        link_states_.pose[i].position.z = p[i].coeff(2);

        link_states_.pose[i].orientation.x = 0.0;
        link_states_.pose[i].orientation.y = 0.0;
        link_states_.pose[i].orientation.z = 0.0;
        link_states_.pose[i].orientation.w = 0.0;
    }
    pub_link_states_.publish(link_states_);
}

void GazeboInterface::rotateLink(const std::vector<Eigen::Quaternion<double>>& q)
{
    if (q.size() != link_states_.name.size())
    {
        std::stringstream msg;
        msg << "q.size() != link_states_.name.size()" << std::endl
        << "  q.size : " << q.size() << std::endl
        << "  link_states_.name.size : " << link_states_.name.size();
        throw ops_wbc_gazebo_interface::Exception("GazeboInterface::rotateLink", msg.str());
    }

    for (uint32_t i = 0; i < q.size(); i++)
    {
        link_states_.pose[i].position.x = 0.0;
        link_states_.pose[i].position.y = 0.0;
        link_states_.pose[i].position.z = 0.0;

        link_states_.pose[i].orientation.x = q[i].x();
        link_states_.pose[i].orientation.y = q[i].y();
        link_states_.pose[i].orientation.z = q[i].z();
        link_states_.pose[i].orientation.w = q[i].w();        
    }
    pub_link_states_.publish(link_states_);
}

const Eigen::VectorXd& GazeboInterface::getJointStates()
{
    ops_wbc_utils::ScopedLock lock(mutex_);

    for (uint32_t i = 0; i < joint_list_.size(); i++)
    {
        if (joint_to_idx_.find(joint_list_[i]) == joint_to_idx_.end())
        {
            std::stringstream msg;
            msg << joint_list_[i] << " was not found in joint_to_idx_.";
            throw ops_wbc_gazebo_interface::Exception("ops_wbc_gazebo_interface::GazeboInterface::getJointStates", msg.str());
        }

        joint_state_[joint_list_[i]]->read(q_[i]);
    }

    return q_;
}

const Eigen::VectorXd& GazeboInterface::getJointTorque()
{
    ops_wbc_utils::ScopedLock lock(mutex_);
    return torque_sensor_->getJointTorque();
}

const Eigen::VectorXd& GazeboInterface::getExternalForce(const std::string& joint_name)
{
    ops_wbc_utils::ScopedLock lock(mutex_);
    return force_sensor_->getExternalForce(joint_name);
}

const std::map<std::string, Eigen::VectorXd>& GazeboInterface::getExternalForce()
{
    ops_wbc_utils::ScopedLock lock(mutex_);
    return force_sensor_->getExternalForce();
}



