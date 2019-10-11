#!/usr/bin/env python
import sys
import copy
import rospy
import tf2_ros
import tf
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection

def main():
	# initialize moveit_commander and rospy
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('go_to_cartesion_pose', anonymous=True)
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	# get position of the apriltag 
	#while not rospy.is_shutdown():
	#	try:		
	#		(position, quaternion) = listener.lookupTransform('tag_4','base',rospy.Time(0))
	#	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	#		continue
	Point = tfBuffer.lookup_transform("base","tag_4", rospy.Time(0), rospy.Duration(10))
	position = Point.transform.translation
	quaternion = Point.transform.rotation
	print position
	print quaternion


	# subscriber, only run for the first message received, spinOnce? need to do
	# sub_once = None
	# rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.apriltag_callback)
	# rospy.spin()

	# instantiation the object of the RobotCommander (robot interface)
	robot = moveit_commander.RobotCommander()

	# instantiation the object of the PlanningSceneInterface (environment around the robot) 
	scene = moveit_commander.PlanningSceneInterface()

	# instantiation the object of the MoveGroupCommander (use the right arm for motion planning)
	group = moveit_commander.MoveGroupCommander("right_arm")

	# create display trajectory publisher, we can visualize the planned path in rviz
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 
							moveit_msgs.msg.DisplayTrajectory)

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



	print "============ Generating plan 1"
	pose_target = geometry_msgs.msg.Pose()
	quaternion_orig = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
	# rotate by 180 about y
	quaternion_rot = tf.transformations.quaternion_from_euler(0, math.pi, 0)
	quaternion_new = tf.transformations.quaternion_multiply(quaternion_rot, quaternion_orig)
	pose_target.orientation.x = quaternion_new[0]
	pose_target.orientation.y = quaternion_new[1]
	pose_target.orientation.z = quaternion_new[2]
	pose_target.orientation.w = quaternion_new[3]   # 1.0
	pose_target.position.x = position.x        # 0.6 
	pose_target.position.y = position.y        # 0.2
	pose_target.position.z = position.z        # 0.8
	
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
	rospy.sleep(10)

	# move the arm
	#group.go(wait=True)
	

if __name__ == '__main__':
	main()
