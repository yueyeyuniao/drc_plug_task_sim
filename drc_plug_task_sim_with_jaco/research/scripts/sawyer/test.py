#!/usr/bin/env python
import sys
import copy
import rospy
import tf2_ros
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
import intera_interface

def main():
	# initialize moveit_commander and rospy
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('go_to_cartesion_pose', anonymous=True)
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	# get position of the apriltag 
	# tag_4
	Point = tfBuffer.lookup_transform("base","tag_4", rospy.Time(0), rospy.Duration(10))	
	position = Point.transform.translation
	quaternion = Point.transform.rotation
	# tag_2
	#Point_2 = tfBuffer.lookup_transform("base","tag_2", rospy.Time(0), rospy.Duration(10))
	#position_2 = Point_2.transform.translation
	#quaternion_2 = Point_2.transform.rotation

	#print position_2

	# instantiation the object of the RobotCommander (robot interface)
	robot = moveit_commander.RobotCommander()

	# instantiation the object of the PlanningSceneInterface (environment around the robot) 
	scene = moveit_commander.PlanningSceneInterface()

	# instantiation the object of the MoveGroupCommander (use the right arm for motion planning)
	group = moveit_commander.MoveGroupCommander("right_arm")

	# create display trajectory publisher, we can visualize the planned path in rviz
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 
							moveit_msgs.msg.DisplayTrajectory)
	# gripper
	gripper = intera_interface.Gripper()

	# wait for rviz
	print "============ Waiting for RVIZ..."
	rospy.sleep(10)

	# print the basic information
	print "============ Reference frame: %s" % group.get_planning_frame()
	print "============ Reference frame: %s" % group.get_end_effector_link()
	print "============ Robot Groups:"
	print robot.get_group_names()
	print "============ Printing robot state"
	print robot.get_current_state()
	print "============"

	# open the gripper
	gripper.open()
	rospy.sleep(2)

	# Create box obstacle
	box_pose = geometry_msgs.msg.PoseStamped()
	#box_pose.header.frame_id = robot.get_planning_frame()
	box_pose.header.frame_id = "base"
	box_pose.pose.position.x = 0.5
	box_pose.pose.position.y = -0.6
	box_pose.pose.position.z = 0.0
	box_pose.pose.orientation.w = 1.0
	box_name = "box"
	scene.add_box(box_name, box_pose, size=(0.5, 0.5, 1.3))
	# Create table obstacle
	table_pose = geometry_msgs.msg.PoseStamped()
	#table_pose.header.frame_id = robot.get_planning_frame()
	table_pose.header.frame_id = "base"
	table_pose.pose.position.x = -0.5
	table_pose.pose.position.y = 0.8
	table_pose.pose.position.z = -0.4
	table_pose.pose.orientation.w = 1.0
	table_name = "table"
	scene.add_box(table_name, table_pose, size=(1.0, -1.0, -0.4))

	print "=============================== Generating plan 1"
	pose_target = geometry_msgs.msg.Pose()
	quaternion_rot = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
	euler = tf.transformations.euler_from_quaternion(quaternion_rot)
	euler_rot_yaw = euler[1]
	euler_rot_roll = euler[0]-3.14
	euler_rot_pitch = euler[2]	
	quaternion_rot = tf.transformations.quaternion_from_euler(euler_rot_roll, euler_rot_yaw, euler_rot_pitch)
	pose_target.orientation.x = quaternion_rot[0]
	pose_target.orientation.y = quaternion_rot[1]
	pose_target.orientation.z = quaternion_rot[2]
	pose_target.orientation.w = quaternion_rot[3]   # 1.0
	pose_target.position.x = position.x-0.05        # 0.6 
	pose_target.position.y = position.y+0.18        # 0.2
	pose_target.position.z = position.z+0.1        # 0.8
	
	group.set_pose_target(pose_target)
	plan1 = group.plan()
	print "============ Waiting while RVIZ displays plan1..."
	rospy.sleep(10)

	print "============ Visualizing plan1"
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()

	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan1)
	display_trajectory_publisher.publish(display_trajectory);

	print "============ Waiting while plan1 is visualized (again)..."
	rospy.sleep(15)


	# if sys.version_info < (3, 0): input = raw_input
	choice = raw_input("1: Move the arm  else: Quit")
	if choice == '1':
		group.go(wait=True)
		# close the gripper
		# gripper.close()
		rospy.sleep(3)
	else:
		group.clear_pose_targets()		
		exit()

	print "=============================== Generating plan 1_2"
	pose_target = geometry_msgs.msg.Pose()
	quaternion_rot = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
	euler = tf.transformations.euler_from_quaternion(quaternion_rot)
	euler_rot_yaw = euler[1]
	euler_rot_roll = euler[0]-3.14
	euler_rot_pitch = euler[2]	
	quaternion_rot = tf.transformations.quaternion_from_euler(euler_rot_roll, euler_rot_yaw, euler_rot_pitch)
	pose_target.orientation.x = quaternion_rot[0]
	pose_target.orientation.y = quaternion_rot[1]
	pose_target.orientation.z = quaternion_rot[2]
	pose_target.orientation.w = quaternion_rot[3]   # 1.0
	pose_target.position.x = position.x+0.08        # 0.6 
	pose_target.position.y = position.y+0.18        # 0.2
	pose_target.position.z = position.z+0.1        # 0.8
	
	group.set_pose_target(pose_target)
	plan_1_2 = group.plan()
	print "============ Waiting while RVIZ displays plan1_2..."
	rospy.sleep(10)

	
	# if sys.version_info < (3, 0): input = raw_input
	choice_1_2 = raw_input("1: Move the arm  else: Quit")
	if choice_1_2 == '1':
		group.go(wait=True)
		# close the gripper
		gripper.close()
		rospy.sleep(3)
	else:
		group.clear_pose_targets()		
		exit()


	print "=============================== Generating plan 2"
	pose_target_2 = geometry_msgs.msg.Pose()
	quaternion_rot_2 = [quaternion_2.x, quaternion_2.y, quaternion_2.z, quaternion_2.w]
	euler_2 = tf.transformations.euler_from_quaternion(quaternion_rot_2)
	euler_rot_yaw_2 = euler_2[1]
	euler_rot_roll_2 = euler_2[0]-3.14
	euler_rot_pitch_2 = euler_2[2]	
	quaternion_rot_2 = tf.transformations.quaternion_from_euler(euler_rot_roll_2, euler_rot_yaw_2, euler_rot_pitch_2)
	pose_target_2.orientation.x = quaternion_rot_2[0]
	pose_target_2.orientation.y = quaternion_rot_2[1]
	pose_target_2.orientation.z = quaternion_rot_2[2]
	pose_target_2.orientation.w = quaternion_rot_2[3]   # 1.0
	pose_target_2.position.x = position_2.x-0.1        # 0.6 
	pose_target_2.position.y = position_2.y        # 0.2
	pose_target_2.position.z = position_2.z+0.1        # 0.8
	
	group.set_pose_target(pose_target_2)
	plan2 = group.plan()
	print "============ Waiting while RVIZ displays plan2..."
	rospy.sleep(10)


	# if sys.version_info < (3, 0): input = raw_input
	choice_2 = raw_input("1: Move the arm  else: Quit")
	if choice_2 == '1':
		group.go(wait=True)
		# close the gripper
		gripper.close()
		rospy.sleep(3)
		group.clear_pose_targets()
	else:
		group.clear_pose_targets()		
		exit()





	## plan 2
	#group.clear_pose_targets()
	#group_variable_values = group.get_current_joint_values()
	#print "============ Joint values: ", group_variable_values
	#group_variable_values[0] = 1.0
	#group.set_joint_value_target(group_variable_values)

	#plan2 = group.plan()

	#print "============ Waiting while RVIZ displays plan2..."
	#rospy.sleep(5)

	## plan 3
	#waypoints = []
	## start with the current pose
	#waypoints.append(group.get_current_pose().pose) # problem with get_current_pose !!!!!!!

	## first orient gripper and move forward (+x)
	#wpose = geometry_msgs.msg.Pose()
	#wpose.orientation.w = 1.0
	#wpose.position.x = waypoints[0].position.x + 0.1
	#wpose.position.y = waypoints[0].position.y
	#wpose.position.z = waypoints[0].position.z
	#waypoints.append(copy.deepcopy(wpose))
	#(plan3, fraction) = group.compute_cartesian_path(
        #                     waypoints,   # waypoints to follow
        #                     0.01,        # eef_step
        #                     0.0)         # jump_threshold

	#print "============ Waiting while RVIZ displays plan3..."
	#rospy.sleep(5)


if __name__ == '__main__':
	main()
