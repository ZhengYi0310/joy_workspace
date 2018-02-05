/*************************************************************************
	> File Name: test.cpp
	> Author:Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Sun 04 Feb 2018 07:56:42 PM PST
 ************************************************************************/

#include <fstream>
#include <ros/ros.h>
#include <utils/exception.hpp>
#include "ops_wbc_robot/robot/parser.hpp"
#include "ops_wbc_robot/robot/tf_publisher.hpp"
#include <ops_wbc_digital_filter/pseudo_differentiator.hpp>

using namespace ops_wbc_robot;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "parser_test");
    ros::NodeHandle nh;

    try
    {
        std::string name = "youbot";
        RobotPtr robot = std::make_shared<Robot>(name);

        ParserPtr parser = std::make_shared<Parser>();
        std::string path = "";
        parser->load(path, robot);

        ros::MultiThreadedSpinner spinner;
        TfPublisherPtr tf_publisher = std::make_shared<TfPublisher>();

        const std::string mnp_name = "mnp";
        unsigned long cnt = 0;
        const double period = 0.001;
        ros::Rate r(1/period);

        ops_wbc_digital_filter::PseudoDifferentiatorPtr differentiator = std::make_shared<ops_wbc_digital_filter::PseudoDifferentiator>(period, 1.0);
        Eigen::VectorXd q = Eigen::VectorXd::Constant(robot->getDOF(mnp_name), 0.0);
        Eigen::VectorXd dq = Eigen::VectorXd::Constant(robot->getDOF(mnp_name), 0.0);
        differentiator->init(q, dq);

        Eigen::VectorXd pre_q = q;
        while (ros::ok())
        {
            q = Eigen::VectorXd::Constant(robot->getDOF(mnp_name), 1.0);
            double coeff = 1.0 * sin(2.0 * M_PI * 0.1 * cnt * period);
            cnt++;

            q = coeff * q;
            q.coeffRef(0) = 0.0;
            q.coeffRef(1) = 0.0;
            q.coeffRef(2) = 0.0;

            robot->update(q);
            robot->computeJacobian(mnp_name);
            robot->computeMassMatrix(mnp_name);

            differentiator->apply(q);

            Eigen::VectorXd dq1 = robot->getJointVelocity(mnp_name);

            Eigen::VectorXd dq2;
            differentiator->copyDerivativeValueTo(dq2);

            std::cout << "p     : " << q.transpose() << std::endl;
            std::cout << "pre_p : " << pre_q.transpose() << std::endl;

            Eigen::VectorXd dq3 = (q - pre_q) / period;
            pre_q = q;

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

