/*************************************************************************
	> File Name: dynamic_movement_primitive_test_1.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 18 Nov 2016 11:27:56 AM PST
 ************************************************************************/

//system includes 
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>

// local includes
#include "dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h"
#include "dynamic_movement_primitive/../../dmpLib/test_dmp/test.h"

using namespace dmp;

static std::string abs_bag_file_name_node_handle = "/tmp/test_dmp_from_node_handle.bag";
static std::string abs_bag_file_name_message = "/tmp/test_dmp_from_node_message.bag";

TEST(dmp_tests, dmp_lib_test)
{
    EXPECT_TRUE(test_dmp::Test::test("/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test/", false));
}

TEST(dmp_tests, initFromNodeHandle)
{
    dmp_lib::NC2010DMPPtr dmp_from_node_handle;
    ros::NodeHandle node_handle("~/joint_space_dmp");

    ROS_INFO("Checking for %s.", node_handle.getNamespace().c_str());

    std::vector<std::string> robot_part_names;
    robot_part_names.push_back("RIGHT_ARM");

    bool result = NC2010DynamicMovementPrimitive::initFromNodeHandle(dmp_from_node_handle, robot_part_names, node_handle);
    EXPECT_TRUE(result);

    result = NC2010DynamicMovementPrimitive::writeToDisc(dmp_from_node_handle, abs_bag_file_name_node_handle);
    EXPECT_TRUE(result);

    dmp_lib::NC2010DMPPtr dmp_from_node_handle_copy;
    result = NC2010DynamicMovementPrimitive::readFromDisc(dmp_from_node_handle_copy, abs_bag_file_name_node_handle);
    EXPECT_TRUE(result);

    dmp_lib::Time initial_time, initial_time_copy;
    double teaching_duration, teaching_duration_copy;
    double execution_duration, execution_duration_copy;
    double cutoff, cutoff_copy;
    int type, type_copy;
    int id, id_copy;

    EXPECT_TRUE(dmp_from_node_handle->getParameters()->get(initial_time, teaching_duration, execution_duration, cutoff, type, id));
    EXPECT_TRUE(dmp_from_node_handle_copy->getParameters()->get(initial_time_copy, teaching_duration_copy, execution_duration_copy, cutoff_copy, type_copy, id_copy));
    EXPECT_TRUE(initial_time == initial_time_copy);
    EXPECT_TRUE(execution_duration == execution_duration_copy);
    EXPECT_TRUE(teaching_duration == teaching_duration_copy);
    EXPECT_TRUE(cutoff == cutoff_copy);
    EXPECT_TRUE(type ==type_copy);
    EXPECT_TRUE(id == id_copy);

    EXPECT_TRUE(*dmp_from_node_handle == *dmp_from_node_handle);
    EXPECT_TRUE(*dmp_from_node_handle == *dmp_from_node_handle_copy);
    EXPECT_FALSE(*dmp_from_node_handle != *dmp_from_node_handle_copy);

    NC2010DMPMsg nc2010_dmp_msg;
    result = NC2010DynamicMovementPrimitive::writeToMessage(dmp_from_node_handle_copy, nc2010_dmp_msg);
    EXPECT_TRUE(result);

    dmp_lib::NC2010DMPPtr dmp_init_from_message;
    result = NC2010DynamicMovementPrimitive::createFromMessage(dmp_init_from_message, nc2010_dmp_msg);
    EXPECT_TRUE(result);

    EXPECT_TRUE(dmp_init_from_message->getParameters()->get(initial_time, teaching_duration, execution_duration, cutoff, type, id));
    EXPECT_TRUE(initial_time == initial_time_copy);
    EXPECT_TRUE(execution_duration == execution_duration_copy);
    EXPECT_TRUE(teaching_duration == teaching_duration_copy);
    EXPECT_TRUE(cutoff == cutoff_copy);
    EXPECT_TRUE(type == type_copy);
    EXPECT_TRUE(id == id_copy);

    result = NC2010DynamicMovementPrimitive::writeToDisc(dmp_init_from_message, abs_bag_file_name_message);
    EXPECT_TRUE(result);
} 

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "dmp_tests");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

