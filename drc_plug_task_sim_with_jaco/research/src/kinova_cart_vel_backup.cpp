// will use the /joint_state for the IK
// will publish the calculated the position/velocity to the /j2n6s300/effort_joint_trajectory_controller/command



#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <kinova_msgs/PoseVelocity.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <string.h>
#include <vector>


class kinovaCartesianVelocity
{
public:
	std::vector<double> joint_states;
	tf::TransformListener listener_;
    tf::StampedTransform transform;
    geometry_msgs::Pose pose_ee;
    geometry_msgs::Pose pose_ee_desired;
    bool robot_model_loaded = false;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::JointModelGroup* joint_model_group;
	std::vector<std::string> joint_names;
	double x_current_ee, y_current_ee, z_current_ee, roll_current_ee, pitch_current_ee, yaw_current_ee;
	double x_desired_ee, y_desired_ee, z_desired_ee, roll_desired_ee, pitch_desired_ee, yaw_desired_ee;
	double x_step_ee, y_step_ee, z_step_ee, roll_step_ee, pitch_step_ee, yaw_step_ee;
	double x_input_ee, y_input_ee, z_input_ee, roll_input_ee, pitch_input_ee, yaw_input_ee;
	double velocityToPosition = 0.05;
	double positionToVelocity = 0.05;
	std::vector<double> joint_speeds;

	ros::NodeHandle n;
	ros::Subscriber joint_state_sub;
	ros::Subscriber cartesian_velocity_sub;
	ros::Publisher joint_velocity_pub;
	kinovaCartesianVelocity();
	void poseVelocity_callback(kinova_msgs::PoseVelocity msg);
	void jointState_callback(sensor_msgs::JointState msg);
	void printJointStates(std::vector<double> joint_states);
};

kinovaCartesianVelocity::kinovaCartesianVelocity()
{	
	joint_state_sub = n.subscribe("/j2n6s300/joint_states", 5, &kinovaCartesianVelocity::jointState_callback, this);
	cartesian_velocity_sub = n.subscribe("/jaco_endeffector_cartesian_velocity", 5, &kinovaCartesianVelocity::poseVelocity_callback, this);
	joint_velocity_pub = n.advertise<trajectory_msgs::JointTrajectory>("/j2n6s300/effort_joint_trajectory_controller/command", 10);	
}


void kinovaCartesianVelocity::jointState_callback(sensor_msgs::JointState msg)
{
	
	joint_states.resize(9);
	joint_states = msg.position;
	
}
void kinovaCartesianVelocity::poseVelocity_callback(kinova_msgs::PoseVelocity msg)
{
	// get the joint state RobotCommander.get_current_state()
	// get the end-effector position /GetPositionFK /TF
	// Scale your Cartesian velocity by a constant and add it onto this position. Call the GetPositionIK service of your move_group to get an inverse kinematics solution for this
	// Subtract your current joint angles from the IK joint angles, and multiply by some constant to get a velocity for every joint

	// get current state
	printJointStates(joint_states);

	// using moveit
	// load the model once
	if (robot_model_loaded == false)
	{
		robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
		robot_model::RobotModelPtr kinematic_model_temp = robot_model_loader.getModel();
		kinematic_model = kinematic_model_temp;
		robot_model_loaded = true;
	}
	// robot state
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	for (int i=0; i<6; i++)
	{
		kinematic_state->setJointPositions("j2n6s300_joint_"+ std::to_string(i+1), &joint_states[i]);	
	}
	joint_model_group = kinematic_model->getJointModelGroup("arm");
	joint_names = joint_model_group->getJointModelNames();

	joint_names.erase(joint_names.begin(), joint_names.begin()+1);
	joint_names.pop_back();

	
	// get joint value
	// std::vector<double> joint_values;
	// kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	// for(std::size_t i = 0; i < joint_names.size(); ++i)
	// {
	//   ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	// }

	// get the ee position using tf
	try{
          listener_.waitForTransform("root", "j2n6s300_end_effector",ros::Time(0), ros::Duration(3.0));
          listener_.lookupTransform("root", "j2n6s300_end_effector",ros::Time(0), transform);
      }
      catch (tf::TransformException ex) {
          ROS_ERROR("%s",ex.what());
      }
    pose_ee.position.x = transform.getOrigin().x();
	pose_ee.position.y = transform.getOrigin().y();
	pose_ee.position.z = transform.getOrigin().z();
	pose_ee.orientation.x = transform.getRotation().x();
	pose_ee.orientation.y = transform.getRotation().y();
	pose_ee.orientation.z = transform.getRotation().z();
	pose_ee.orientation.w = transform.getRotation().w();

	//std::cout << "we getting the pose ee" << pose_ee.position.x << ", " << pose_ee.position.y << ", " << pose_ee.position.z << std::endl;

	// get the desired ee position
	// get the current
	tf::Quaternion q(pose_ee.orientation.x,pose_ee.orientation.y,pose_ee.orientation.z,pose_ee.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll_current_ee, pitch_current_ee, yaw_current_ee); 
    x_current_ee = pose_ee.position.x;
    y_current_ee = pose_ee.position.y;
    z_current_ee = pose_ee.position.z;

    // get the step
	// the step is in the ee frame, we need to transfer to root frame
    tf::Quaternion desiredQuaternion_ee;
	desiredQuaternion_ee.setRPY(msg.twist_angular_x, msg.twist_angular_y, msg.twist_angular_z);

	geometry_msgs::PoseStamped ee_pose, root_pose;
  	ee_pose.header.frame_id = "j2n6s300_end_effector";
  	ee_pose.pose.position.x = msg.twist_linear_x;
  	ee_pose.pose.position.y = msg.twist_linear_y;
  	ee_pose.pose.position.z = msg.twist_linear_z;
  	ee_pose.pose.orientation.x = desiredQuaternion_ee.x();
  	ee_pose.pose.orientation.y = desiredQuaternion_ee.y();
  	ee_pose.pose.orientation.z = desiredQuaternion_ee.z();
  	ee_pose.pose.orientation.w = desiredQuaternion_ee.w();

	try{
	    listener_.waitForTransform("root", "j2n6s300_end_effector",ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("root", "j2n6s300_end_effector",ros::Time(0), transform);
	    listener_.transformPose("root", ee_pose, root_pose);
	  }
	  catch(tf::TransformException& ex){
	    ROS_ERROR("Received an exception trying to transform a pose %s", ex.what());
	  }

	// transform quaternion to euler
	tf::Quaternion q_input(root_pose.pose.orientation.x,root_pose.pose.orientation.y,root_pose.pose.orientation.z,root_pose.pose.orientation.w);
    tf::Matrix3x3 m_input(q_input);
    m_input.getRPY(roll_input_ee, pitch_input_ee, yaw_input_ee); 
    x_input_ee = root_pose.pose.position.x;
    y_input_ee = root_pose.pose.position.y;
    z_input_ee = root_pose.pose.position.z;

    x_step_ee = x_input_ee * velocityToPosition;
    y_step_ee = y_input_ee * velocityToPosition;
    z_step_ee = z_input_ee * velocityToPosition;
    roll_step_ee = roll_input_ee * velocityToPosition;
    pitch_step_ee = pitch_input_ee * velocityToPosition;
    yaw_step_ee = yaw_input_ee * velocityToPosition;
    // get the desired
    x_desired_ee = x_current_ee + x_step_ee;
    y_desired_ee = y_current_ee + y_step_ee;
    z_desired_ee = z_current_ee + z_step_ee;
    roll_desired_ee = roll_current_ee + roll_step_ee;
    pitch_desired_ee = pitch_current_ee + pitch_step_ee;
    yaw_desired_ee = yaw_current_ee + yaw_step_ee;
    // transfer to quaternion
	tf::Quaternion desiredQuaternion;
	desiredQuaternion.setRPY(roll_desired_ee, pitch_desired_ee, yaw_desired_ee);
	pose_ee_desired.position.x = x_desired_ee;
	pose_ee_desired.position.y = y_desired_ee;
	pose_ee_desired.position.z = z_desired_ee;
	pose_ee_desired.orientation.x = desiredQuaternion.x();
	pose_ee_desired.orientation.y = desiredQuaternion.y();
	pose_ee_desired.orientation.z = desiredQuaternion.z();
	pose_ee_desired.orientation.w = desiredQuaternion.w();

 
	// IK
	std::vector<double> joint_values;
	label:
	bool found_ik = kinematic_state->setFromIK(joint_model_group, pose_ee_desired, 10, 0.1);
	if (found_ik)
	{	  	
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		for(std::size_t i = 0; i < joint_names.size(); ++i)
		{
		  ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
		}
		ROS_INFO("Find IK solution");	  
	}
	else
	{
		ROS_INFO("Did not find IK solution");
		goto label;
	}
	
	// calculate the joint velocity
	joint_speeds.resize(6);
	joint_speeds[0] = joint_values[0] - joint_states[0]; // First joint is not right
	for (int i=1; i<6; i++)
	{
		joint_speeds[i] = joint_values[i] - joint_states[i];
	}

	// print cout the speed for debuging
	for (int i=0; i<6; i++)
	{
		std::cout << "joint" << i+1 << "speeds: " << joint_speeds[i] << std::endl;
	}

	// publish the speed
	trajectory_msgs::JointTrajectory traj;
	traj.header.stamp = ros::Time::now();
	traj.header.frame_id = "j2n6s300_end_effector";
	traj.joint_names.resize(6);
	traj.points.resize(1);
	traj.points[0].positions.resize(6);
	traj.points[0].velocities.resize(6);

	traj.joint_names = joint_names;

	for (int i=0; i<6; i++)
	{
		traj.points[0].positions[i] = joint_states[i]+joint_speeds[i]*velocityToPosition;
		traj.points[0].velocities[i] = joint_speeds[i];
	}
	traj.points[0].time_from_start = ros::Duration(1);
	joint_velocity_pub.publish(traj);



}

void kinovaCartesianVelocity::printJointStates(std::vector<double> joint_states)
{
	for (int i=0; i<9; i++)
	{
		std::cout << "Joint" << i+1 <<" = " << joint_states[i] << std::endl; 
	}
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "Velocity_control");
	kinovaCartesianVelocity obj;
	ros::spin();
	return 0;

}