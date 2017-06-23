//This file is used to bring wam to start position of the task us trac_ik inverse kinematics solver
#include <boost/date_time.hpp>
#include <iostream>
#include <vector>
#include <iterator>
#include <algorithm>
#include <cmath>

#include <ros/ros.h>

#include <trac_ik/trac_ik.hpp>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chain.hpp>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>

#include <wam_msgs/JointMove.h>
#include <wam_msgs/RTJointPos.h> // the real time joint position message
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Empty.h>

trajectory_msgs::JointTrajectory create_joint_traj(double start_pos[7], double end_pos[7], double duration_of_traj, double frequency_of_traj)
{
	/*
    Create a trajectory from start_position to end_position.
    The trajectory is the linear interpolation from start to end. It will last
    duration_of_trajectory seconds.  Be careful that you pick your start/end
    points such that the hand doesn't turn into the arm.
    args:
      start_position: a 7 element array containing the start position for each
        joint.
      end_position: a 7 element array containing the end position for each
        joint.
      duration: a float containing the time the trajectory should finish at
        in seconds.
      frequency_of_trajectory: the frequency of the trajectory you want to
        generate in Hz.
    returns:
      trajectory: a trajectory_msgs::JointTrajectory type of N x 7 points
      vel_lims: the joint velocities required to perform trajectory.
        v1 = abs((p2 - p1) * 250). The last velocity, vn, is set equal to v(n-1)
        The absolute value is required for the WAM. Even if it's traveling in
        the negative direction, a positive value should be used.
    */
    double freq_of_msgs = frequency_of_traj; // in hz
    double num_of_way_points = duration_of_traj * freq_of_msgs;

   	trajectory_msgs::JointTrajectory joint_traj;
   	joint_traj.points.resize(num_of_way_points);
   	

   	double interpolation_step[7];
   	for (int i = 0; i < 7; i++)
   	{
   		interpolation_step[i] = (end_pos[i] - start_pos[i]) / num_of_way_points;

   	}
   	// feed in the joint positions to the trajectory
   	for (int i = 0; i < num_of_way_points; i++)
   	{

   		for (int j = 0; j < 7; j++)
   		{
   			joint_traj.points[i].positions.resize(7);
   			joint_traj.points[i].velocities.resize(7);
   			joint_traj.points[i].accelerations.resize(7);
   			joint_traj.points[i].positions[j] = start_pos[j] + interpolation_step[j] * i; 
   		}
   	}

   	// feed in the joint trajectory velocities to the trajectory
   	for (int i = 0; i < num_of_way_points - 1; i++)
   	{
   		for (int j = 0; j < 7; j++)
   		{
   			joint_traj.points[i].velocities[j] = std::abs((joint_traj.points[i].positions[j + 1] - joint_traj.points[i].positions[j]) * frequency_of_traj);
   			if (joint_traj.points[i].velocities[j] > 1.0)
   			{
   				ROS_ERROR("One or more of the values in the specified velocities"
                          "Exceed 1 rad / second. The robot won't like this."
                          "Adjust the trajectory so that each point can be "
						  "reached without exceeding this limit.");
   				break;
   			}
   		}	
   	}
   	// Because this is discrete differentiation,
    // the last value is missing: len(vel_lims) = len(trajectory) - 1
	// so we just repeat the last calculated velocity.
	joint_traj.points[num_of_way_points - 1].velocities.assign(joint_traj.points[num_of_way_points - 2].velocities.begin(), joint_traj.points[num_of_way_points - 2].velocities.end());
	return joint_traj;
}

/*
void send_joint_traj(ros::NodeHandle nh, trajectory_msgs::JointTrajectory joint_traj, double frequency = 250.0)
{
	"""
    This is used to send a trajectory to the WAM arm at a given frequency.
    args:
      joint_traj: a trajectory_msgs::JointTrajectory type with Nx7 points. The 7 columns correspond to the 7 joints
        on a WAM.
      frequency: The frequency the trajectory should be published at.
    returns:
      None.
    """

    ros::Publisher rt_joint_pos_pub = nh.advertise<wam_msgs::RTJointPos>("/jnt_pos_cmd", 100);
    //If wam_node is running, it will be connected to this publisher.
    //Mostly this loop is here because you want to make sure the publisher
	//gets set up before it starts sending information.
	while (rt_joint_pos_pub.getNumSubscribers() < 1)
	{
		ROS_INFO("Waiting for the publisher to go up.")
		ros::Duration(0.5).sleep();
	}

	double traj_len = joint_traj.points.size();
	bool finished = false;
	int traj_row = 0;
	wam_msgs::RTJointPos msg_for_service;

	ros::Rate loop_rate(frequency);

	while (nh.ok() && (not finished))
	{
		msg_for_service.joints.assign(joint_traj.points[traj_row].positions, joint_traj.points[traj_row].positions + 7);
		msg_for_service.rate_limits.assign(joint_traj.points[traj_row].velocities, joint_traj.velocities[traj_row].velocities + 7);
		traj_row++;
		rt_joint_pos_pub.publish(msg_for_service);

		if (traj_row = joint_traj.points.size() - 1)
		{
			finished = tree;
		}

		loop_rate.sleep();
	}

}
*/

KDL::JntArray computeIK(ros::NodeHandle& nh, double start_joint_pos[7], std::string chain_start, std::string chain_end, double timeout, std::string urdf_param)
{
	double eps = 1e-6;

	// This constructor parses the URDF loaded in rosparm urdf_param into the
  	// needed KDL structures.  
  	TRAC_IK::TRAC_IK  trac_ik_solver(chain_start, chain_end, urdf_param, timeout, eps);

  	KDL::Chain chain;
  	KDL::JntArray ll, ul; //lower joint limits, upper joint limits
  	
  	bool valid = trac_ik_solver.getKDLChain(chain);
  	if (!valid)
  	{
  		ROS_ERROR("There is no valid KDL chain found");
  	}

  	valid = trac_ik_solver.getKDLLimits(ll, ul);
  	if (!valid)
  	{
  		ROS_ERROR("There are no valid KDL joint limits found");
  	}

  	assert(chain.getNrOfJoints() == ll.data.size());
  	assert(chain.getNrOfJoints() == ul.data.size());
  	ROS_INFO("Using %d joints", chain.getNrOfJoints());

  	// Set up KDL IK
  	KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
  	KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
  	KDL::ChainIkSolverPos_NR_JL kdl_solver(chain,ll,ul,fk_solver, vik_solver, 1, eps); // Joint Limit Solver
  // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK) 

  	// bring up the tf transform listener 
  	tf::TransformListener listener;
  	tf::StampedTransform transform;
  	tf::StampedTransform transform_camera_start;

  	try
  	{
  		// Neeeeeeeeeeeeeeever use ros::Time(now) in tf::listener !!!!!!!!!!!!!!!!11
  		while(!listener.waitForTransform("start_frame", "left_wam/base_link", ros::Time(0.0), ros::Duration(1.5)) && ros::ok()){}
  		listener.lookupTransform("start_frame", "left_wam/base_link", ros::Time(0.0), transform);
  		ROS_INFO_STREAM("child frame: " << transform.child_frame_id_ << " frame: " << transform.frame_id_);
  	}

  	catch (tf::TransformException ex)
  	{
  	    ROS_ERROR("%s",ex.what());
  	    ros::Duration(1.0).sleep();
  	}

  	// convert start joint position to KDL JntArray
  	KDL::JntArray init_jpos(chain.getNrOfJoints());
  	if (init_jpos.data.size() != 7)
  	{
  		ROS_ERROR("Size of start joint pose and end joint pose if different!");
  	}

  	for (unsigned int i = 0; i < init_jpos.data.size(); i++)
  	{
  			init_jpos(i) = start_joint_pos[i];
  			ROS_INFO_STREAM("Value for " << i << " th joint: " << start_joint_pos[i]);
  	}

  	// convert the tf StampedTransformation to a KDL Frame
	KDL::Frame end_effector_pose;
	//tf::transformTFToKDL(transform, end_effector_pose);
	//////////////////
	start_joint_pos[0] = start_joint_pos[0] - 0.2;
	//start_joint_pos[1] = start_joint_pos[1] + 0.3;
	KDL::JntArray q(chain.getNrOfJoints());
	for (unsigned int i = 0; i < q.data.size(); i++)
  	{
  			q(i) = start_joint_pos[i];
  			ROS_INFO_STREAM("Value for " << i << " th joint: " << start_joint_pos[i]);
  	}

	fk_solver.JntToCart(q,end_effector_pose);
    ///////////////////////////////

	KDL::JntArray end_jpos(chain.getNrOfJoints());

	boost::posix_time::ptime start_time;
  	boost::posix_time::time_duration diff;
  	int rc;

	// compute the inverse kinematics 
	ROS_INFO("*** Computing Inverse Kinematics Using TRAC-IK! ***");
	double elapsed = 0;
    start_time = boost::posix_time::microsec_clock::local_time();
    rc=trac_ik_solver.CartToJnt(init_jpos, end_effector_pose, end_jpos);
    diff = boost::posix_time::microsec_clock::local_time() - start_time;
    elapsed = diff.total_nanoseconds() / 1e9;
    if (rc>=0)
    {
    	ROS_INFO_STREAM("TRAC-IK found a solution in " << elapsed << " seconds!");
    	return end_jpos;
    }
    else
    {
    	ROS_ERROR("No solution found!");
    }

}

int main(int argc, char** argv)
{
  srand(1);
  ros::init(argc, argv, "ik_tests");
  ros::NodeHandle nh("~");

  // register the service client for the joint move
  ros::ServiceClient joint_move = nh.serviceClient<wam_msgs::JointMove>("/joint_move");
  ros::ServiceClient go_home = nh.serviceClient<std_srvs::Empty>("/go_home");

  // register the publisher of the real time joint position message
  ros::Publisher rt_joint_pos_pub = nh.advertise<wam_msgs::RTJointPos>("/jnt_pos_cmd", 100);

  /*
  while (nh.ok())
  {
  	ros::spinOnce();
  	tf::StampedTransform transform;
  	tf::TransformListener listener;
  	try
  	{
  		while(!listener.waitForTransform("left_wam/base_link", "start_frame", ros::Time(0.0), ros::Duration(1.5)) && ros::ok()){}
  		listener.lookupTransform("left_wam/base_link", "start_frame", ros::Time(0.0), transform);
  		ROS_INFO_STREAM("child frame: " << transform.child_frame_id_ << " frame: " << transform.frame_id_);
  	}

  	catch (tf::TransformException &ex) 
  	{       
  	     ROS_ERROR("%s",ex.what());
  		 ros::Duration(1.0).sleep();
  	}
  }
  */

  std::string chain_start, chain_end, urdf_param;
  double timeout;

  nh.param("chain_start", chain_start, std::string("left_wam/base_link"));
  nh.param("chain_end", chain_end, std::string("left_wam/wrist_palm_link"));
  
  if (chain_start=="" || chain_end=="") {
    ROS_FATAL("Missing chain info in launch file");
    exit (-1);
  }

  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));

  ROS_INFO("Node is initialized!");
  
  ros::Duration(5.0).sleep();
  ROS_INFO("Bring wam to the initial position!");

  // get the wam for the pre-start position
  double startPoint[] = {1.60, -1.36, 0.00, -0.38, -0.01, -0.28, -0.03};
  double rt_rate_limit[] = {0, 0, 0, 0, 0, 0, 0};
  wam_msgs::JointMove jointMovePos;
  wam_msgs::RTJointPos rt_joint_pos;
  std_srvs::Empty empty;
  jointMovePos.request.joints.assign(startPoint, startPoint + 7);
  // assign the real time position message
  rt_joint_pos.joints.assign(startPoint, startPoint + 7);
  rt_joint_pos.rate_limits.assign(rt_rate_limit, rt_rate_limit + 7);
  
  
  ros::Rate loop_rate(50);
  while (nh.ok())
  {
  	ros::spinOnce();
  	ROS_INFO("*****Publish the realtime joint position message!*****");
  	rt_joint_pos_pub.publish(rt_joint_pos);

  	loop_rate.sleep();
  }
  
  /*
  joint_move.waitForExistence();
  ROS_INFO("*******Joint Move Service Available!");
  
  if(joint_move.call(jointMovePos))
  {
  	ROS_INFO("*******Joint Move Succeeds!");
  }
  else
  {
  	ROS_WARN("*******Callback failed!");
  }
  */
  
  /*
  ros::Duration(5.0).sleep();
  ROS_INFO("Wait for 10 secs! Bring wam to the start position!");

  KDL::JntArray end_jpos = computeIK(nh, startPoint, chain_start, chain_end, timeout, urdf_param);
  double endPoint[7];

  for (unsigned int i = 0; i < end_jpos.data.size(); i++)
  {
  	endPoint[i] = end_jpos(i);
  }
  ros::Duration(10.0).sleep();
  jointMovePos.request.joints.assign(endPoint, endPoint + 7);
  joint_move.call(jointMovePos);
  ROS_INFO("Wam reaches the start position!");
  */


  return 0;
}