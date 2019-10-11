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

#include <cmath>        // std::abs

using namespace std;
const double FINGER_MAX = 6400;
geometry_msgs::Pose pose_new;

class SurpriseTask
{
  public:
    SurpriseTask()
    {
        display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    
    }


    void getTargetPose()
    {
      // get the position of the wire
        listener_.waitForTransform("/root", "wire",ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("/root", "wire",ros::Time(0), transform_wire);

        target_pose_wire1.position.x = transform_wire.getOrigin().x()-0.05;
        target_pose_wire1.position.y = transform_wire.getOrigin().y()+0.15;
        target_pose_wire1.position.z = transform_wire.getOrigin().z()+0.02;
        target_pose_wire1.orientation.x = transform_wire.getRotation().x(); 
        target_pose_wire1.orientation.y = transform_wire.getRotation().y(); 
        target_pose_wire1.orientation.z = transform_wire.getRotation().z(); 
        target_pose_wire1.orientation.w = transform_wire.getRotation().w(); 

        target_pose_wire2 = target_pose_wire1;
        target_pose_wire2.position.y = target_pose_wire1.position.y-0.095;   
        
        target_pose_wire3 = target_pose_wire2;
        target_pose_wire3.position.y = target_pose_wire2.position.y+0.13;

        target_pose_wire4 = target_pose_wire3;
        target_pose_wire4.position.x = target_pose_wire3.position.x+0;


        std::cout << "Position of target pose: " << transform_wire.getOrigin().x() << ", " << transform_wire.getOrigin().y() << ", " << transform_wire.getOrigin().z() << std::endl;
    }

    void getTipPose()
    {
        listener_.waitForTransform("socket2", "wire_tip",ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("socket2", "wire_tip",ros::Time(0), transform_tip);


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
        ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", delta_roll, delta_pitch, delta_yaw);
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

      // set the planner and palnning attempts
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
      sleep(2.0);

      // Visualizing plans
      if (1)
      {
        ROS_INFO("Visualizing plan (again)");    
        display_trajectory.trajectory_start = my_plan.start_state_;
        display_trajectory.trajectory.push_back(my_plan.trajectory_);
        display_publisher.publish(display_trajectory);
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(2.0);
      }
      
      // move the robot 
      ROS_INFO("Attention: moving the arm");
      // gripper_action(0.0); // open the gripper
      group.move();
      sleep(1.0);
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

      // set the planner and palnning attempts
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
      sleep(2.0);

      // Visualizing plans
      if (1)
      {
        ROS_INFO("Visualizing plan (again)");    
        display_trajectory.trajectory_start = my_plan.start_state_;
        display_trajectory.trajectory.push_back(my_plan.trajectory_);
        display_publisher.publish(display_trajectory);
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(2.0);
      }
      
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

      sleep(1.0);
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


      // set the planner and palnning attempts
      group.setPlannerId("RRTConnectkConfigDefault");
      group.setNumPlanningAttempts(100);

      // Planning to a Pose goal
      group.setPoseTarget(target_pose_wire1);


      // Now, we call the planner to compute the plan and visualize it.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
      bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
      /* Sleep to give Rviz time to visualize the plan. */
      sleep(2.0);

      // Visualizing plans
      if (1)
      {
        ROS_INFO("Visualizing plan 1 (again)");    
        display_trajectory.trajectory_start = my_plan.start_state_;
        display_trajectory.trajectory.push_back(my_plan.trajectory_);
        display_publisher.publish(display_trajectory);
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(2.0);
      }
      
      // move the robot 
      ROS_INFO("Attention: moving the arm");
      gripper_action(0.0); // open the gripper
      group.move();
      sleep(1.0);
  ////////////////////////////////////////////////////////////////////////////////////////
      // move the robot with the object
      group.setPoseTarget(target_pose_wire2);

      // Now, we call the planner to compute the plan and visualize it.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
      // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
      bool success2 = (group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 2 (pose goal) %s",success2?"":"FAILED");    
      /* Sleep to give Rviz time to visualize the plan. */
      sleep(2.0);

      // Visualizing plans
      if (1)
      {
        ROS_INFO("Visualizing plan 2 (again)");    
        display_trajectory.trajectory_start = my_plan2.start_state_;
        display_trajectory.trajectory.push_back(my_plan2.trajectory_);
        display_publisher.publish(display_trajectory);
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(2.0);
      }


      ROS_INFO("Attention: moving the arm with the object");
      group.move();
      gripper_action(FINGER_MAX); // close the gripper
      sleep(1.0);
  /////////////////////////////////////////////////////////////////////////////////////////
      group.setPoseTarget(target_pose_wire3);

      // Now, we call the planner to compute the plan and visualize it.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
      // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
      bool success3 = (group.plan(my_plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 3 (pose goal) %s",success3?"":"FAILED");    
      /* Sleep to give Rviz time to visualize the plan. */
      sleep(2.0);

      // Visualizing plans
      if (1)
      {
        ROS_INFO("Visualizing plan 3 (again)");    
        display_trajectory.trajectory_start = my_plan3.start_state_;
        display_trajectory.trajectory.push_back(my_plan3.trajectory_);
        display_publisher.publish(display_trajectory);
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(2.0);
      }


      ROS_INFO("Attention: moving the arm with the object");
      group.move();
      sleep(1.0);
  ///////////////////////////////////////////////////////////////////////////////////////////////
      group.setPoseTarget(target_pose_wire4);

      // Now, we call the planner to compute the plan and visualize it.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan4;
      // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
      bool success4 = (group.plan(my_plan4) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 4 (pose goal) %s",success4?"":"FAILED");    
      /* Sleep to give Rviz time to visualize the plan. */
      sleep(2.0);

      // Visualizing plans
      if (1)
      {
        ROS_INFO("Visualizing plan 4 (again)");    
        display_trajectory.trajectory_start = my_plan4.start_state_;
        display_trajectory.trajectory.push_back(my_plan4.trajectory_);
        display_publisher.publish(display_trajectory);
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(2.0);
      }


      ROS_INFO("Attention: moving the arm with the object");
      group.move();
      sleep(1.0);
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

      listener_.waitForTransform("socket2", "wire_tip",ros::Time(0), ros::Duration(3.0));
      listener_.lookupTransform("socket2", "wire_tip",ros::Time(0), transform_tip);


      delta_x = transform_tip.getOrigin().x();
      delta_y = transform_tip.getOrigin().y();
      delta_z = transform_tip.getOrigin().z();
      std::cout << "relative positions between tip and socket2: " << delta_x << "," << delta_y << "," << delta_z;
      // get the position of the endeffector
      listener_.waitForTransform("/root", "j2n6s300_end_effector",ros::Time(0), ros::Duration(3.0));
      listener_.lookupTransform("/root", "j2n6s300_end_effector",ros::Time(0), transform_ee);


      pose_new.position.x = transform_ee.getOrigin().x() + delta_y + 0.02;
      pose_new.position.y = transform_ee.getOrigin().y() + delta_x + 0.05;
      pose_new.position.z = transform_ee.getOrigin().z() + delta_z; // -0.1 for avoiding the socket2


      std::cout << "Position for the pre-insert: " << pose_new.position.x << "," << pose_new.position.y << "," << pose_new.position.z << std::endl;
      pose_new.orientation.x = transform_ee.getRotation().x();
      pose_new.orientation.y = transform_ee.getRotation().y();
      pose_new.orientation.z = transform_ee.getRotation().z();
      pose_new.orientation.w = transform_ee.getRotation().w();


      Move_robot_once_test(pose_new);

      pose_new.position.x = pose_new.position.x - 0.05;
      Move_robot_once_test(pose_new);
    }

    void wire_terminal_insertion()
    {
      pose_new.position.y = pose_new.position.y - 0.02;
      Move_robot_once(pose_new);
      pose_new.position.y = pose_new.position.y - 0.06;
      Move_robot_once(pose_new);
      gripper_action(0.0); // open the gripper
      pose_new.position.y = pose_new.position.y + 0.08;
      Move_robot_once(pose_new);

    }

    void move_to_ready_position()
    {
      geometry_msgs::Pose ready_pose;

      // this is for testing orientation alignment function
      // ready_pose.position.x = 0.385;    //0.286 -> power cable  0.385 -> hdmi wire
      // ready_pose.position.y = -0.3;    // -0.23 -> power cable  -0.3 -> hdmi wire
      // ready_pose.position.z = 0.45;    // 0.383 -> power cable  0.45 -> hdmi wire
      // ready_pose.orientation.x = -0.659; 
      // ready_pose.orientation.y = 0.669; 
      // ready_pose.orientation.z = -0.240; 
      // ready_pose.orientation.w = -0.247;

      // this it for testing pose alignment function with disturbances socket 2
      // ready_pose.position.x = 0.034;    
      // ready_pose.position.y = -0.364;   
      // ready_pose.position.z = 0.5;     //0.412
      // ready_pose.orientation.x = 0.740; 
      // ready_pose.orientation.y = -0.600; 
      // ready_pose.orientation.z = 0.241; 
      // ready_pose.orientation.w = 0.183;

      // socket 1
      ready_pose.position.x = 0.384;    
      ready_pose.position.y = -0.364;   
      ready_pose.position.z = 0.412;     //0.412
      ready_pose.orientation.x = 0.740; 
      ready_pose.orientation.y = -0.600; 
      ready_pose.orientation.z = 0.241; 
      ready_pose.orientation.w = 0.183;

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

      // set the planner and palnning attempts
      group.setPlannerId("RRTConnectkConfigDefault");
      group.setNumPlanningAttempts(100);

      // Planning to a Pose goal
      group.setPoseTarget(ready_pose);


      // Now, we call the planner to compute the plan and visualize it.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
      bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan (pose goal) %s",success?"":"FAILED");    
      /* Sleep to give Rviz time to visualize the plan. */
      sleep(2.0);

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
      //gripper_action(0.0); // open the gripper
      group.move();
      gripper_action(FINGER_MAX); // close the gripper
      sleep(1.0);
    }


  private:
    ros::NodeHandle nh_;
    ros::Publisher display_publisher;

    tf::TransformListener listener_;
    tf::StampedTransform transform_wire;
    tf::StampedTransform transform_tip;
    tf::StampedTransform transform_ee;

    geometry_msgs::Pose target_pose_wire1, target_pose_wire2, target_pose_wire3, target_pose_wire4;
    double delta_roll, delta_pitch, delta_yaw; 
    double delta_x, delta_y, delta_z;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Surprise_Task");
    ros::AsyncSpinner spinner(4);  // important
    spinner.start();

    SurpriseTask ST;
    ROS_INFO("Initialized");


    // experiment setup
    ST.move_to_ready_position();

    // real code
    // ST.getTargetPose();
    // ST.Grasp_and_Pullout();

    // adjust the tip pose - new function
    // ST.adjust_tip_orientation_auto();

    // wire-terminal insertion
    // ST.wire_terminal_pre_insertion();
    // ST.wire_terminal_insertion();
    return 0;
}