/*************************************************************************
	> File Name: test2.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 05 Feb 2018 04:27:07 PM PST
 ************************************************************************/

#include <ros/ros.h>
#include <utils/exception.hpp>
#include <ops_wbc_robot/robot/parser.hpp>
#include <ops_wbc_robot/robot/tf_publisher.hpp>

using namespace ops_wbc_robot;

Eigen::MatrixXd M;
Eigen::MatrixXd J0, J1, J2;
Eigen::MatrixXd T0, T1, T2;

void calc()
{
    Eigen::Matrix2d m;
    double m1 = 3.0;
    double m2 = 1.5;
    double l1 = 0.5;
    double l2 = 0.4;
    double th1 = -M_PI / 2.0;
    double th2 =  M_PI / 2.0;
    double Izz1 = 0.05;
    double Izz2 = 0.025;
    m.coeffRef(0, 0) = m1 * l1 * l1 + Izz1 + m2 * (l1 * l1 + l2 * l2 + 2 * l1 * l2 * cos(th2)) + Izz2;
    m.coeffRef(0, 1) = m2 * (l2 * l2 + l1 * l2 * cos(th2)) + Izz2;
    m.coeffRef(1, 0) = m2 * (l2 * l2 + l1 * l2 * cos(th2)) + Izz2;
    m.coeffRef(1, 1) = l2 * l2 * m2 + Izz2;

    Eigen::Matrix2d j;
    j.coeffRef(0, 0) = -l1 * sin(th1) - l2 * sin(th1 + th2);
    j.coeffRef(0, 1) = -l2 * sin(th1 + th2);
    j.coeffRef(1, 0) = l1 * cos(th1) + l2 * cos(th1 + th2);
    j.coeffRef(1, 1) = l2 * cos(th1 + th2);

    std::cout << "M : " << std::endl << M << std::endl << std::endl
              << "m : " << std::endl << m << std::endl << std::endl
              << "Jv0 : " << std::endl << J0.block(0, 0, 2, 2) << std::endl
              << "Ref : " << std::endl << -l1 * sin(th1) << std::endl
              << l1 * cos(th1) << std::endl
              << "Jv1 : " << std::endl << J1.block(0, 0, 2, 2) << std::endl
              << "Ref : " << std::endl
              << -l1 * sin(th1) - l2* sin(th1 + th2) << " "
              << -l2 * sin(th1 + th2) << std::endl
              << l1 * cos(th1) + l2 * cos(th1 + th2) << " "
              << l2 * cos(th1 + th2) << std::endl << std::endl
              << "Jv2 : " << std::endl << J2.block(0, 0, 2, 2) << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "parset_test");
    ros::NodeHandle nh;

    try 
    {
        std::string name = "rr_robot";
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
            q.coeffRef(0) = -M_PI / 2.0;
            q.coeffRef(1) =  M_PI / 2.0;

            robot->update(q);
            M = robot->getMassMatrix(mnp_name);
            J0 = robot->getJacobian(mnp_name, "link1");
            J1 = robot->getJacobian(mnp_name, "link2");
            J2 = robot->getJacobian(mnp_name, "gripper");
            ManipulatorPtr mnp = robot->getManipulator(mnp_name);

            calc();
            tf_publisher->publish(robot,false);
            r.sleep();
        }
    }
    catch (ops_wbc_utils::Exception& e)
    {
        ROS_ERROR_STREAM(e.what());
    }

    return 0;
}

