/*************************************************************************
	> File Name: test_dmp.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 04 Jan 2017 02:20:58 PM PST
 ************************************************************************/

// system includes 
#include <ros/package.h>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

#include <skill_library/getAffordance>
#include <skill_library/Affordance>

#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_utilities.h>
#include <robot_info/robot_info.h>

// local includes 
#include "dmp_behaviors/test_dmp.h"
#include "dmp_behaviors/behavior_utilities"

using namespace dmp_behavior_actions;
using namespace skill_library;

namespace dmp_behaviors
{
    TestDMP::TestDMP(ros::Nodehandle& node_handle, const std::string& action_name) : node_handle_(node_handle), action_server_(ros::Nodehandle("/Behaviors"), action_name, boost::bind(&TestDMP::execute. this, _1), false), skill_library_client_(node_handle)
    {
        std::string robot_part, base_frame, controller_namespace;
        ROS_VERIFY(usc_utilities::read(node_handle, "robot_part", robot_part));
        ROS_VERIFY(usc_utilities::read(node_handle, "base_frame", base_frame));
        ROS_VERIFY(usc_utilities::read(node_handle, "controller_namespace", controller_namespace));
        ROS_VERIFY(readParams());

        std::vector<std::string> dmp_controller_names;
        dmp_controller_names.push_back(dmp_controller_name_);

        ROS_VERIFY(dmp_controller_client_.initialize(robot_part, dmp_controller_names, controller_namespace));
        dmp_controller_client_.setSingleThreadedMode(false);
    }

    void TestDMP::start()
    {
        action_server_.start();
    }

    void TestDMP::execute(const dmp_behavior_actions::TestDMPGoalConstPtr& goal)
    {

        if (!readParams())
        {
            BehaviorUtilities<TestDMP, ActionServer>::failed("Problems when reading parameters.", action_server_);
            return;
        }

        if (goal->dmp_bag_file_name.empty())
        {
            BehaviorUtilities<TestDMP, ActionServer>::failed("No DMP bag file name is specified.", action_server_);
            return;
        }

        dmp_lib::DMPBasePtr dmp;
        if (!skill_library_client_.get(goal->dmp_bag_file_name, dmp))
        {
            BehaviorUtilities<TestDMP, ActionServer>::failed("Could not get DMP from skill_library.", action_server_);
            return;
        }

        if(!dmp->setup())
        {
            BehaviorUtilities<TestDMP, ActionServer>::failed("Could not setup DMP.", action_server_);
            return;
        }

        if(!dmp_controller_client_.sendCommand(dmp, dmp_controller_name_, true))
        {
            BehaviorUtilities<TestDMP, ActionServer>::failed("Could send DMP to controller.", action_server_);
            return;
        }

        TestDMPResult result;
        result.result = TestDMPResult::SUCCEEDED;
        action_server_.setSucceeded(result);
    }

    bool TestDMP::readParams()
    {
        ROS_VERIFY(usc_utilities::read(node_handle_, "dmp_controller_name", dmp_controller_name_));
        return true;
    }
}

