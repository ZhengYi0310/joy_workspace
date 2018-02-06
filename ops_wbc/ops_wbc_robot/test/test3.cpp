/*************************************************************************
	> File Name: test3.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 05 Feb 2018 04:49:03 PM PST
 ************************************************************************/

#include <ros/ros.h>
#include <utils/exception.hpp>
#include <ops_wbc_robot/robot/parser.hpp>
#include <ops_wbc_robot/robot/tf_publisher.hpp>
using namespace ops_wbc_robot;

Eigen::MatrixXd M;
Eigen::MatrixXd J;
Eigen::MatrixXd T0, T1, T2;

void calc()
{
    Eigen::MatrixXd M_inv = M.inverse();
    Eigen::MatrixXd Jv = J.block(0, 0, 3, J.cols());
    Eigen::MatrixXd Lambda_inv = Jv * M_inv * Jv.transpose();
    Eigen::MatrixXd Lambda = Lambda_inv.inverse();
    Eigen::MatrixXd J_dyn_inv = M_inv * Jv.transpose() * Lambda;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
    Eigen::MatrixXd N = I - J_dyn_inv * Jv;

    std::cout << N.transpose() << std::endl << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "parser_test");
    ros::NodeHandle nh;

    try
    {
        std::string name = "lwr";
        RobotPtr robot = std::make_shared<Robot>(name);

        ParserPtr parser = std::make_shared<Parser>();
        std::string path = "";
        parser->load(path, robot);

        ros::MultiThreadedSpinner spinner;
        TfPublisherPtr tf_publisher = std::make_shared<TfPublisher>();
        const std::string mnp_name = "mnp";
        unsigned long cnt = 0;
        ros::Rate r(10.0);

        while (ros::ok())
        {
            Eigen::VectorXd q = Eigen::VectorXd::Zero(robot->getDOF(mnp_name));
            ManipulatorPtr mnp = robot->getManipulator(mnp_name);

            double goal = sin(2.0 * M_PI * 0.2 * cnt * 0.1);
            cnt++;
            for (uint32_t i = 0; i < q.rows(); i++)
            {
                q.coeffRef(i) = M_PI / 4.0 * goal;
            }

            q = Eigen::VectorXd::Constant(q.rows(), M_PI / 4.0);

            robot->update(q);
            robot->computeJacobian(mnp_name);
            robot->computeMassMatrix(mnp_name);
            M = robot->getMassMatrix(mnp_name);
            J = robot->getJacobian(mnp_name, "gripper");

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
