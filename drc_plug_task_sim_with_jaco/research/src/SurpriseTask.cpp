#include <iostream>
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <kinova_driver/kinova_ros_types.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <geometry_msgs/PointStamped.h>
#include <kinova_msgs/PoseVelocity.h>  // velocity
#include <std_msgs/Float32.h>

#include <cmath>        // std::abs
#include <ctime>
#include <chrono>
using namespace std;
const double FINGER_MAX = 6400;
geometry_msgs::Pose pose_new;

class SurpriseTask
{
  public:
    SurpriseTask()
    {
        display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        velocity_publisher = nh_.advertise<kinova_msgs::PoseVelocity>("/j2n6s300_driver/in/cartesian_velocity", 1, true);
    }


    void positionController(float x, float y, float z, float roll, float pitch, float yaw)
    {
        kinova_msgs::PoseVelocity msg;
        float k = 3; 
        float k_r = 5;
        
        msg.twist_linear_x = k*x;
        msg.twist_linear_y = k*y;
        msg.twist_linear_z = k*z;
        msg.twist_angular_x = k_r*roll;
        msg.twist_angular_y = k_r*pitch;
        msg.twist_angular_z = k_r*yaw;


        velocity_publisher.publish(msg);

        // ros::Time beginTime = ros::Time::now();
        // ros::Duration secondsIWantToSendMessagesFor = ros::Duration(3); 
        // ros::Time endTime = secondsIWantToSendMessagesFor + beginTime;
        // while(ros::Time::now() < endTime )
        // {
        //     pub.publish(msg);

        //     // Time between messages, so you don't blast out an thousands of 
        //     // messages in your 3 secondperiod
        //     ros::Duration(0.1).sleep();
        // }
    }

        void velocity_pub(float x, float y, float z, float roll, float pitch, float yaw)
    {
        kinova_msgs::PoseVelocity msg;       
        msg.twist_linear_x = x;
        msg.twist_linear_y = y;
        msg.twist_linear_z = z;
        msg.twist_angular_x = roll;
        msg.twist_angular_y = pitch;
        msg.twist_angular_z = yaw;


        velocity_publisher.publish(msg);

        // ros::Time beginTime = ros::Time::now();
        // ros::Duration secondsIWantToSendMessagesFor = ros::Duration(3); 
        // ros::Time endTime = secondsIWantToSendMessagesFor + beginTime;
        // while(ros::Time::now() < endTime )
        // {
        //     pub.publish(msg);

        //     // Time between messages, so you don't blast out an thousands of 
        //     // messages in your 3 secondperiod
        //     ros::Duration(0.1).sleep();
        // }
    }

    void getTargetPose()
    {
      // get the position of the wire
        listener_.waitForTransform("/root", "cable",ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("/root", "cable",ros::Time(0), transform_wire);

        target_pose_wire1.position.x = transform_wire.getOrigin().x()-0.05;
        target_pose_wire1.position.y = transform_wire.getOrigin().y()+0.15;
        target_pose_wire1.position.z = transform_wire.getOrigin().z()+0.02;
        target_pose_wire1.orientation.x = transform_wire.getRotation().x(); 
        target_pose_wire1.orientation.y = transform_wire.getRotation().y(); 
        target_pose_wire1.orientation.z = transform_wire.getRotation().z(); 
        target_pose_wire1.orientation.w = transform_wire.getRotation().w(); 

        target_pose_wire2 = target_pose_wire1;
        target_pose_wire2.position.y = target_pose_wire1.position.y-0.085;   
        
        target_pose_wire3 = target_pose_wire2;
        target_pose_wire3.position.y = target_pose_wire2.position.y+0.15;


        //std::cout << "Position of target pose: " << transform_wire.getOrigin().x() << ", " << transform_wire.getOrigin().y() << ", " << transform_wire.getOrigin().z() << std::endl;
    }

    void getTipPose()
    {
        listener_.waitForTransform("socket2", "cable_tip",ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("socket2", "cable_tip",ros::Time(0), transform_tip);


        delta_x = transform_tip.getOrigin().x();
        delta_y = transform_tip.getOrigin().y();
        delta_z = transform_tip.getOrigin().z();

        // double delta_x_socket = transform_tip.getOrigin().x();
        // double delta_y_socket = transform_tip.getOrigin().y();
        // double delta_z_socket = transform_tip.getOrigin().z();

        // // transform points from socket frame to root frame 
        // geometry_msgs::PointStamped points_root, points_socket;
        // points_socket.header.frame_id = "socket2";
        // points_socket.header.stamp = ros::Time();
        // points_socket.point.x = delta_x_socket;
        // points_socket.point.y = delta_y_socket;
        // points_socket.point.z = delta_z_socket;
        // listener_.transformPoint("/root", points_socket, points_root);

        // delta_x = points_root.point.x;
        // delta_y = points_root.point.y;
        // delta_z = points_root.point.z;
        // ROS_INFO("x, y, z=%1.2f  %1.2f  %1.2f", delta_x, delta_y, delta_z);


        tf::Quaternion q_tip(transform_tip.getRotation().x(),transform_tip.getRotation().y(),transform_tip.getRotation().z(),transform_tip.getRotation().w());
        tf::Matrix3x3 m_tip(q_tip);
        m_tip.getRPY(delta_roll, delta_pitch, delta_yaw);
        //ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", delta_roll, delta_pitch, delta_yaw);
        // get the position of the endeffector
        listener_.waitForTransform("/root", "j2n6s300_end_effector",ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("/root", "j2n6s300_end_effector",ros::Time(0), transform_ee);
        tf::Quaternion q_ee(transform_ee.getRotation().x(),transform_ee.getRotation().y(),transform_ee.getRotation().z(),transform_ee.getRotation().w());
        tf::Matrix3x3 m_ee(q_ee);
        double roll_ee, pitch_ee, yaw_ee;
        m_ee.getRPY(roll_ee, pitch_ee, yaw_ee);
        // get the target rotation
        double roll_target, pitch_target, yaw_target;
        roll_target = roll_ee + delta_roll;
        pitch_target = pitch_ee + delta_pitch;
        yaw_target = yaw_ee + delta_yaw;
        tf::Quaternion q_new = tf::createQuaternionFromRPY(roll_target, pitch_target, yaw_target); 

        pose_new.position.x = transform_ee.getOrigin().x();
        pose_new.position.y = transform_ee.getOrigin().y();
        pose_new.position.z = transform_ee.getOrigin().z();
        pose_new.orientation.x = q_new.x();
        pose_new.orientation.y = q_new.y();
        pose_new.orientation.z = q_new.z();
        pose_new.orientation.w = q_new.w();

    }

    bool gripper_action(double finger_turn)
    {

      if (finger_turn < 0)
      {
          finger_turn = 0.0;
      }
      else
      {
          finger_turn = std::min(finger_turn, FINGER_MAX);
      }

      kinova_msgs::SetFingersPositionGoal goal;
      goal.fingers.finger1 = finger_turn;
      goal.fingers.finger2 = goal.fingers.finger1;
      goal.fingers.finger3 = goal.fingers.finger1;

      actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> finger_client("/j2n6s300_driver/fingers_action/finger_positions" , false);
      while(!finger_client.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the finger action server to come up");
      } 


      finger_client.sendGoal(goal);

      if (finger_client.waitForResult(ros::Duration(5.0)))
      {
          finger_client.getResult();
          return true;
      }
      else
      {
          finger_client.cancelAllGoals();
          ROS_WARN_STREAM("The gripper action timed-out");
          return false;
      }
    }

    void Move_robot_once(geometry_msgs::Pose pose)
    {
      moveit::planning_interface::MoveGroupInterface group("arm");
      moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
     
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

      // Create a publisher for visualizing plans in Rviz.
      moveit_msgs::DisplayTrajectory display_trajectory;

      // Getting Basic Information
      // We can print the name of the reference frame for this robot.
      ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
      
      // We can also print the name of the end-effector link for this group.
      ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

      // Planning to a Pose goal
      group.setPoseTarget(pose);


      // Now, we call the planner to compute the plan and visualize it.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
      bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan (pose goal) %s",success?"":"FAILED");    
      /* Sleep to give Rviz time to visualize the plan. */
      // sleep(2.0);

      // Visualizing plans
      // if (1)
      // {
      //   ROS_INFO("Visualizing plan (again)");    
      //   display_trajectory.trajectory_start = my_plan.start_state_;
      //   display_trajectory.trajectory.push_back(my_plan.trajectory_);
      //   display_publisher.publish(display_trajectory);
      //   /* Sleep to give Rviz time to visualize the plan. */
      //   sleep(2.0);
      // }
      
      // move the robot 
      ROS_INFO("Attention: moving the arm");
      // gripper_action(0.0); // open the gripper
      group.move();
      // sleep(1.0);
    }

    void Move_robot_once_test(geometry_msgs::Pose pose)
    {
      moveit::planning_interface::MoveGroupInterface group("arm");
      moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
     
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

      // Create a publisher for visualizing plans in Rviz.
      moveit_msgs::DisplayTrajectory display_trajectory;

      // Getting Basic Information
      // We can print the name of the reference frame for this robot.
      ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
      
      // We can also print the name of the end-effector link for this group.
      ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

      group.setPlannerId("RRTConnectkConfigDefault");
      group.setNumPlanningAttempts(100);

      // Planning to a Pose goal
      group.setPoseTarget(pose);


      // Now, we call the planner to compute the plan and visualize it.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
      bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan (pose goal) %s",success?"":"FAILED");    
      /* Sleep to give Rviz time to visualize the plan. */
      // sleep(2.0);

      // Visualizing plans
      // if (1)
      // {
      //   ROS_INFO("Visualizing plan (again)");    
      //   display_trajectory.trajectory_start = my_plan.start_state_;
      //   display_trajectory.trajectory.push_back(my_plan.trajectory_);
      //   display_publisher.publish(display_trajectory);
      //   /* Sleep to give Rviz time to visualize the plan. */
      //   sleep(2.0);
      // }
      
      // move the robot 
      ROS_INFO("Attention: moving the arm");
      // gripper_action(0.0); // open the gripper

      int decision;
      std::cout << "make a decision : 1 - run, 0 - no: " << std::endl;
      std::cin >> decision;
      if (decision == 1) 
      {
        group.move();
      }
      else{
        Move_robot_once_test(pose);
      }

      // sleep(1.0);
    }


    void Grasp_and_Pullout()
    { 
      // ros::AsyncSpinner spinner(4);  // important
      // spinner.start();
      //sleep(10.0);
      
      moveit::planning_interface::MoveGroupInterface group("arm");
      moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
     
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

      // Create a publisher for visualizing plans in Rviz.
      moveit_msgs::DisplayTrajectory display_trajectory;

      // Getting Basic Information
      // We can print the name of the reference frame for this robot.
      ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
      
      // We can also print the name of the end-effector link for this group.
      ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

      // Planning to a Pose goal
      group.setPoseTarget(target_pose_wire1);


      // Now, we call the planner to compute the plan and visualize it.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
      bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
      /* Sleep to give Rviz time to visualize the plan. */
      // sleep(2.0);

      // Visualizing plans
      // if (1)
      // {
      //   ROS_INFO("Visualizing plan 1 (again)");    
      //   display_trajectory.trajectory_start = my_plan.start_state_;
      //   display_trajectory.trajectory.push_back(my_plan.trajectory_);
      //   display_publisher.publish(display_trajectory);
      //   /* Sleep to give Rviz time to visualize the plan. */
      //   sleep(2.0);
      // }
      
      // move the robot 
      ROS_INFO("Attention: moving the arm");
      gripper_action(0.0); // open the gripper
      group.move();
      // sleep(1.0);
  ////////////////////////////////////////////////////////////////////////////////////////
      // move the robot with the object
      group.setPoseTarget(target_pose_wire2);

      // Now, we call the planner to compute the plan and visualize it.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
      // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
      bool success2 = (group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 2 (pose goal) %s",success2?"":"FAILED");    
      /* Sleep to give Rviz time to visualize the plan. */
      // sleep(2.0);

      // Visualizing plans
      // if (1)
      // {
      //   ROS_INFO("Visualizing plan 2 (again)");    
      //   display_trajectory.trajectory_start = my_plan2.start_state_;
      //   display_trajectory.trajectory.push_back(my_plan2.trajectory_);
      //   display_publisher.publish(display_trajectory);
      //   /* Sleep to give Rviz time to visualize the plan. */
      //   sleep(2.0);
      // }


      ROS_INFO("Attention: moving the arm with the object");
      group.move();
      gripper_action(FINGER_MAX); // close the gripper
      // sleep(1.0);
  /////////////////////////////////////////////////////////////////////////////////////////
      group.setPoseTarget(target_pose_wire3);

      // Now, we call the planner to compute the plan and visualize it.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
      // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
      bool success3 = (group.plan(my_plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 3 (pose goal) %s",success3?"":"FAILED");    
      /* Sleep to give Rviz time to visualize the plan. */
      // sleep(2.0);

      // Visualizing plans
      // if (1)
      // {
      //   ROS_INFO("Visualizing plan 3 (again)");    
      //   display_trajectory.trajectory_start = my_plan3.start_state_;
      //   display_trajectory.trajectory.push_back(my_plan3.trajectory_);
      //   display_publisher.publish(display_trajectory);
      //   /* Sleep to give Rviz time to visualize the plan. */
      //   sleep(2.0);
      // }


      ROS_INFO("Attention: moving the arm with the object");
      group.move();
    }   

    void adjust_tip_orientation_auto()
    {
      getTipPose();
      while (abs(delta_roll) > 0.02 || abs(delta_pitch) > 0.02 || abs(delta_yaw) > 0.02 )
      {
        Move_robot_once(pose_new);
        getTipPose();
      }
    }

    void wire_terminal_pre_insertion()
    {

      listener_.waitForTransform("socket2", "cable_tip",ros::Time(0), ros::Duration(3.0));
      listener_.lookupTransform("socket2", "cable_tip",ros::Time(0), transform_tip);


      delta_x = transform_tip.getOrigin().x();
      delta_y = transform_tip.getOrigin().y();
      delta_z = transform_tip.getOrigin().z();
      //std::cout << "relative positions between tip and socket2: " << delta_x << "," << delta_y << "," << delta_z;
      // get the position of the endeffector
      listener_.waitForTransform("/root", "j2n6s300_end_effector",ros::Time(0), ros::Duration(3.0));
      listener_.lookupTransform("/root", "j2n6s300_end_effector",ros::Time(0), transform_ee);


      // pose_new.position.x = transform_ee.getOrigin().x() + delta_y;
      // pose_new.position.y = transform_ee.getOrigin().y() + delta_x;
      // pose_new.position.z = transform_ee.getOrigin().z() + delta_z - 0.11; // -0.1 for avoiding the socket2


      // std::cout << "Position for the pre-insert: " << pose_new.position.x << "," << pose_new.position.y << "," << pose_new.position.z << std::endl;
      // pose_new.orientation.x = transform_ee.getRotation().x();
      // pose_new.orientation.y = transform_ee.getRotation().y();
      // pose_new.orientation.z = transform_ee.getRotation().z();
      // pose_new.orientation.w = transform_ee.getRotation().w();


      // Move_robot_once_test(pose_new);

      // pose_new.position.z = pose_new.position.z + 0.11;
      // Move_robot_once_test(pose_new);

      pose_new.position.x = transform_ee.getOrigin().x() + delta_y;
      pose_new.position.y = transform_ee.getOrigin().y() + delta_x + 0.1;
      pose_new.position.z = transform_ee.getOrigin().z() + delta_z - 0.2; // -0.1 for avoiding the socket2


      //std::cout << "Position for the pre-insert: " << pose_new.position.x << "," << pose_new.position.y << "," << pose_new.position.z << std::endl;
      pose_new.orientation.x = transform_ee.getRotation().x();
      pose_new.orientation.y = transform_ee.getRotation().y();
      pose_new.orientation.z = transform_ee.getRotation().z();
      pose_new.orientation.w = transform_ee.getRotation().w();


      Move_robot_once(pose_new);
      pose_new.position.z = pose_new.position.z + 0.2;
      Move_robot_once(pose_new);

    }

    void wire_terminal_pre_insertion_ready()
    {
      number_of_adjustment = number_of_adjustment + 1;
      getTipPose(); // get pose_new orientation
      listener_.waitForTransform("socket2", "cable_tip",ros::Time(0), ros::Duration(3.0));
      listener_.lookupTransform("socket2", "cable_tip",ros::Time(0), transform_tip);


      delta_x = transform_tip.getOrigin().x();
      delta_y = transform_tip.getOrigin().y();
      delta_z = transform_tip.getOrigin().z();


      //std::cout << "relative positions between tip and socket2: " << delta_x << "," << delta_y << "," << delta_z;
      // get the position of the endeffector
      listener_.waitForTransform("/root", "j2n6s300_end_effector",ros::Time(0), ros::Duration(3.0));
      listener_.lookupTransform("/root", "j2n6s300_end_effector",ros::Time(0), transform_ee);

      pose_new.position.x = transform_ee.getOrigin().x() + delta_y;
      pose_new.position.y = transform_ee.getOrigin().y() + delta_x + 0.1; //pre-insert
      pose_new.position.z = transform_ee.getOrigin().z() + delta_z;


      //std::cout << "Position for the pre-insert: " << pose_new.position.x << "," << pose_new.position.y << "," << pose_new.position.z << std::endl;
      //std::cout << "Orientation for the pre-insert: " << pose_new.orientation.x << "," << pose_new.orientation.y << "," << pose_new.orientation.z << std::endl;

      Move_robot_once(pose_new);
      std::cout << "Pose error data:" << delta_x + 0.1<< " " << delta_y << " " << delta_z << " " << delta_roll << " " << delta_pitch << " " << delta_yaw << std::endl;
      std::cout << "Number of adjustments:" << number_of_adjustment << std::endl;
      while (abs(delta_roll) > 0.02 || abs(delta_pitch) > 0.02 || abs(delta_yaw) > 0.02 || abs(delta_x > 0.01) || abs(delta_y > 0.08) || abs(delta_z > 0.01))
      {
        wire_terminal_pre_insertion_ready();
        //std::cout << "Pose error data:" << delta_x << " " << delta_y << " " << delta_z << " " << delta_roll << " " << delta_pitch << " " << delta_yaw << std::endl;
      }

    }

    void updateControlInput()
    {
      listener_.waitForTransform("socket2", "cable_tip",ros::Time(0), ros::Duration(3.0));
      listener_.lookupTransform("socket2", "cable_tip",ros::Time(0), transform_tip);
      tf::Quaternion q_tip(transform_tip.getRotation().x(),transform_tip.getRotation().y(),transform_tip.getRotation().z(),transform_tip.getRotation().w());
      tf::Matrix3x3 m_tip(q_tip);
      m_tip.getRPY(delta_roll, delta_pitch, delta_yaw);
      std::cout << "Pose error before convertion" << delta_x << " " << delta_y << " " << delta_z << " " << delta_roll << " " << delta_pitch << " " << delta_yaw << std::endl;
 

      delta_x = transform_tip.getOrigin().x();
      delta_y = transform_tip.getOrigin().y();   // to the pre-insert state
      delta_z = transform_tip.getOrigin().z();
    }

    void updateControlInput_world()
    {
      listener_.waitForTransform("root", "cable_tip",ros::Time(0), ros::Duration(3.0));
      listener_.lookupTransform("root", "cable_tip",ros::Time(0), transform_tip_world);
      tf::Quaternion q_tip(transform_tip_world.getRotation().x(),transform_tip_world.getRotation().y(),transform_tip_world.getRotation().z(),transform_tip_world.getRotation().w());
      tf::Matrix3x3 m_tip(q_tip);
      m_tip.getRPY(roll_world, pitch_world, yaw_world);
    }

    float pid_calculation(float setpoint, float pv, std::string ss)
    {
      if (ss == "x"){
        _Kp = 2;      // 2
        _Kd = 0.2;    // 0.2
        _Ki = 0; 
      }
      else if (ss=="y"){
        _Kp = 2;      // 2
        _Kd = 0.2;    // 0.2
        _Ki = 0; 
      }
      else if (ss=="z"){
        _Kp = 2;      // 2
        _Kd = 0.2;    // 0.2
        _Ki = 0; 
      }
      else if (ss=="roll"){
        _Kp = 2;      // 2
        _Kd = 0.2;    // 0.2
        _Ki = 0;
      }
      else if (ss=="pitch"){
        _Kp = 2;      // 2
        _Kd = 0.2;    // 0.2
        _Ki = 0;
      }
      else if (ss=="yaw"){
        _Kp = 2;      // 2
        _Kd = 0.2;    // 0.2
        _Ki = 0;
      }


      // Calculate error
      float error = pv - setpoint;

      // Proportional term
      float Pout = _Kp * error;

      float Iout;

      // Integral term
      if (ss == "x"){
        _integral_x += error * _dt;
        Iout= _Ki * _integral_x;
      }
      else if (ss=="y"){
        _integral_y += error * _dt;
        Iout = _Ki * _integral_y;
      }
      else if (ss=="z"){
        _integral_z += error * _dt;
        Iout = _Ki * _integral_z;
      }
      else if (ss=="roll"){
        _integral_roll += error * _dt;
        Iout = _Ki * _integral_roll;
      }
      else if (ss=="pitch"){
        _integral_pitch += error * _dt;
        Iout = _Ki * _integral_pitch;
      }
      else if (ss=="yaw"){
        _integral_yaw += error * _dt;
        Iout = _Ki * _integral_yaw;
      }


      // Derivative term
      float derivative;
      if (ss=="x"){
        derivative = (error - _pre_error_x) / _dt;
      }
      else if (ss=="y"){
        derivative = (error - _pre_error_y) / _dt;
      }
      else if (ss=="z"){
        derivative = (error - _pre_error_z) / _dt;
      }
      else if (ss=="roll"){
        derivative = (error - _pre_error_roll) / _dt;
      }
      else if (ss=="pitch"){
        derivative = (error - _pre_error_pitch) / _dt;
      }
      else if (ss=="yaw"){
        derivative = (error - _pre_error_yaw) / _dt;
      }


      float Dout = _Kd * derivative;

      // Calculate total output
      float output = Pout + Iout + Dout;

      // Save error to previous error
      if (ss=="x"){
        _pre_error_x = error;
      }
      else if (ss=="y"){
        _pre_error_y = error;
      }
      else if (ss=="z"){
        _pre_error_z = error;
      }
      else if (ss=="roll"){
        _pre_error_roll = error;
      }
      else if (ss=="pitch"){
        _pre_error_pitch = error;
      }
      else if (ss=="yaw"){
        _pre_error_yaw = error;
      }
      

      return output;
    }


    void wire_terminal_pre_insertion_ready_with_position_control()
    {
      updateControlInput();
      updateControlInput_world();
      while (abs(delta_roll) > 0.02 || abs(delta_pitch) > 0.02 || abs(delta_yaw) > 0.02 || abs(delta_x > 0.005) || abs(delta_y > 0.105) || abs(delta_z > 0.005))
      {

        // can not use the euler angle directly as angular velocity
        float e_roll = -(float)delta_pitch;
        float e_pitch = -(float)delta_roll;
        float e_yaw = -(float)delta_yaw;


        float a_roll = e_yaw*sin(pitch_world)*sin(roll_world) + e_pitch*cos(roll_world);
        float a_pitch = e_yaw*sin(pitch_world)*cos(roll_world) - e_pitch*sin(roll_world);
        float a_yaw = e_yaw*cos(pitch_world) + e_roll;



        positionController((float)delta_y,((float)delta_x+0.1),((float)delta_z),a_roll,a_pitch,a_yaw);
        //std::cout << "Imhere";
        updateControlInput();
        //wire_terminal_pre_insertion_ready_with_position_control();
        std::cout << "Pose error data:" << delta_x << " " << delta_y << " " << delta_z << " " << delta_roll << " " << delta_pitch << " " << delta_yaw << std::endl;
      }
   

    }


    void wire_terminal_pre_insertion_ready_with_position_control_pid()
    {
      updateControlInput();
      updateControlInput_world();

      while (abs(delta_roll) > 0.05 || abs(delta_pitch) > 0.05 || abs(delta_yaw) > 0.05 || abs(delta_x) > 0.15 || abs(delta_y) > 0.01 || abs(delta_z) > 0.01)
      {
        


        float e_roll = -(float)delta_pitch;
        float e_pitch = -(float)delta_roll;
        float e_yaw = -(float)delta_yaw;

        
        float a_roll = e_yaw*sin(pitch_world)*sin(roll_world) + e_pitch*cos(roll_world);
        float a_pitch = e_yaw*sin(pitch_world)*cos(roll_world) - e_pitch*sin(roll_world);
        float a_yaw = e_yaw*cos(pitch_world) + e_roll;

        // seems incorrect
        // float a_roll = e_roll*cos(yaw_world)*sin(pitch_world) - e_pitch*sin(yaw_world);
        // float a_pitch = e_roll*sin(yaw_world)*sin(pitch_world) + e_pitch*cos(yaw_world);
        // float a_yaw = e_roll*cos(pitch_world) + e_yaw;

        float control_step_x = (float) pid_calculation(0.00, (delta_y),"x");
        float control_step_y = (float) pid_calculation(0.00, (delta_x+0.1),"y");
        float control_step_z = (float) pid_calculation(0.00, (delta_z),"z");
        float control_step_roll = (float) pid_calculation(0.00, (a_roll),"roll");
        float control_step_pitch = (float) pid_calculation(0.00, (a_pitch),"pitch");
        float control_step_yaw = (float) pid_calculation(0.00, (a_yaw),"yaw");
        velocity_pub(control_step_x,control_step_y,control_step_z,control_step_roll,control_step_pitch,control_step_yaw);
        updateControlInput();
        std::cout << "Pose error data:" << delta_x << " " << delta_y << " " << delta_z << " " << delta_roll << " " << delta_pitch << " " << delta_yaw << std::endl;

      }
    }

    void wire_terminal_insertion()
    { 

      listener_.waitForTransform("/root", "j2n6s300_end_effector",ros::Time(0), ros::Duration(3.0));
      listener_.lookupTransform("/root", "j2n6s300_end_effector",ros::Time(0), transform_ee);

      pose_new.position.x = transform_ee.getOrigin().x();
      pose_new.position.y = transform_ee.getOrigin().y() - 0.06;
      pose_new.position.z = transform_ee.getOrigin().z();
      pose_new.orientation.x = transform_ee.getRotation().x();
      pose_new.orientation.y = transform_ee.getRotation().y();
      pose_new.orientation.z = transform_ee.getRotation().z();
      pose_new.orientation.w = transform_ee.getRotation().w();

      Move_robot_once(pose_new);
      pose_new.position.y = pose_new.position.y - 0.11;
      Move_robot_once(pose_new);
      gripper_action(0.0); // open the gripper
      pose_new.position.y = pose_new.position.y + 0.09;
      Move_robot_once(pose_new);

    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher display_publisher;
    ros::Publisher velocity_publisher;

    tf::TransformListener listener_;
    tf::StampedTransform transform_wire;
    tf::StampedTransform transform_tip;
    tf::StampedTransform transform_ee;
    tf::StampedTransform transform_tip_world;

    geometry_msgs::Pose target_pose_wire1, target_pose_wire2, target_pose_wire3;
    double delta_roll, delta_pitch, delta_yaw; 
    double delta_x, delta_y, delta_z;
    double roll_world, pitch_world, yaw_world;
    int number_of_adjustment;

    //pid
    float _dt = 0.1;
    float _Kp;      // 2
    float _Kd;    // 0.2
    float _Ki;    
    float _pre_error_x=0, _pre_error_y=0, _pre_error_z=0, _pre_error_roll=0, _pre_error_pitch=0, _pre_error_yaw=0;
    float _integral_x=0, _integral_y=0, _integral_z=0, _integral_roll=0, _integral_pitch=0, _integral_yaw=0;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Surprise_Task");
    ros::AsyncSpinner spinner(4);  // important
    spinner.start();

    SurpriseTask ST;
    ROS_INFO("Initialized");
    auto start = std::chrono::system_clock::now();  // time 

    ST.getTargetPose();
    ST.Grasp_and_Pullout();

    // adjust the tip pose - new function
    // ST.adjust_tip_orientation_auto();

    // wire-terminal insertion
    // ST.wire_terminal_pre_insertion();
    // ST.wire_terminal_pre_insertion_ready();
    // ST.wire_terminal_insertion();




    ST.wire_terminal_pre_insertion_ready_with_position_control_pid();
    // ST.wire_terminal_pre_insertion_ready();
    ST.wire_terminal_insertion();

    auto end = std::chrono::system_clock::now();     // time

    std::chrono::duration<double> elapsed_seconds = end-start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    std::cout << "finished computation at " << std::ctime(&end_time)
              << "elapsed time: " << elapsed_seconds.count() << "s\n";
    return 0;
}