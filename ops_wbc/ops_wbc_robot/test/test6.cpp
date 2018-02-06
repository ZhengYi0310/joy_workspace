/*************************************************************************
	> File Name: test6.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 05 Feb 2018 05:14:34 PM PST
 ************************************************************************/

#include <ros/ros.h>
#include <utils/exception.hpp>
#include "ops_wbc_robot/robot/parser.hpp"
#include "ops_wbc_robot/robot/tf_publisher.hpp"

using namespace ops_wbc_robot;

Eigen::MatrixXd M1, M2;
Eigen::MatrixXd J1, J2;

void calc()
{
    Eigen::MatrixXd M_inv1 = M1.inverse();
    Eigen::MatrixXd M_inv2 = M2.inverse();
    Eigen::MatrixXd Jv1 = J1.block(0, 0, 3, J1.cols());
    Eigen::MatrixXd Jv2 = J2.block(0, 0, 3, J2.cols());
    Eigen::MatrixXd Lambda_inv1 = Jv1 * M_inv1 * Jv1.transpose();
    Eigen::MatrixXd Lambda_inv2 = Jv2 * M_inv2 * Jv2.transpose();
    Eigen::MatrixXd Lambda1 = Lambda_inv1.inverse();
    Eigen::MatrixXd Lambda2 = Lambda_inv2.inverse();
    Eigen::MatrixXd J_dyn_inv1 = M_inv1 * Jv1.transpose() * Lambda1;
    Eigen::MatrixXd J_dyn_inv2 = M_inv2 * Jv2.transpose() * Lambda2;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(M_inv1.rows(), M_inv1.rows());
    Eigen::MatrixXd N1 = I - J_dyn_inv1 * Jv1;
    Eigen::MatrixXd N2 = I - J_dyn_inv2 * Jv2;

    //std::cout << N.transpose() << std::endl << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "parser_test");
    ros::NodeHandle nh;

    try
    {
        std::string name = "personal_robot";
        RobotPtr robot = std::make_shared<Robot>(name);

        ParserPtr parser = std::make_shared<Parser>();
        std::string path = "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ahl_robot/ahl_robot/yaml/pr2.yaml";
        parser->load(path, robot);

        ros::MultiThreadedSpinner spinner;

        TfPublisherPtr tf_publisher = std::make_shared<TfPublisher>();

        const std::string mnp_name1 = "left_mnp";
        const std::string mnp_name2 = "right_mnp";
        unsigned long cnt = 0;
        ros::Rate r(10.0);

        while(ros::ok())
        {
            Eigen::VectorXd q = Eigen::VectorXd::Zero(robot->getDOF());
            ManipulatorPtr left_mnp = robot->getManipulator(mnp_name1);

            double goal = sin(2.0 * M_PI * 0.2 * cnt * 0.1);
            ++cnt;

            //std::cout << M_PI / 4.0 * goal << std::endl;
            q = Eigen::VectorXd::Constant(q.rows(), 0.0);

            q[10] = goal;

            robot->update(q);
            //robot->update(mnp_name2, q);

            robot->computeJacobian(mnp_name1);
            robot->computeJacobian(mnp_name2);
            robot->computeMassMatrix(mnp_name1);
            robot->computeMassMatrix(mnp_name2);

            M1 = robot->getMassMatrix(mnp_name1);
            M2 = robot->getMassMatrix(mnp_name2);
            J1 = robot->getJacobian(mnp_name1, "gripper_l_link");
            J2 = robot->getJacobian(mnp_name2, "gripper_r_link");

            calc();
            tf_publisher->publish(robot, false);
            r.sleep();
        }
    }
    catch(ops_wbc_utils::Exception& e)
    {
        ROS_ERROR_STREAM(e.what());
    }

    return 0;
}

