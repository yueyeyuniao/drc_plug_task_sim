#!/usr/bin/env python
import sys
import copy
import rospy
import tf2_ros
import tf2_msgs.msg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection

class MoveRobot():
	def __init__(self):
		self.position = None
		self.quaternion = None
		# initialize moveit_commander and rospy
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('go_to_cartesion_pose', anonymous=True)

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
		rospy.sleep(5)

		# print the basic information
		print "============ Reference frame: %s" % group.get_planning_frame()
		print "============ Reference frame: %s" % group.get_end_effector_link()
		print "============ Robot Groups:"
		print robot.get_group_names()
		print "============ Printing robot state"
		print robot.get_current_state()
		print "============"

	def main(self):
		# get position of the apriltag
		tfBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(tfBuffer)
		(self.position, self.quaternion) = tfBuffer.lookup_transform('tag_3','world',rospy.Time(0))
		print position

		print "============ Generating plan 1"
		pose_target = geometry_msgs.msg.Pose()
		pose_target.orientation.w = 1.0
		pose_target.position.x = 0.6
		pose_target.position.y = 0.2
		pose_target.position.z = 0.8
		group.set_pose_target(pose_target)
		plan1 = group.plan()
		print "============ Waiting while RVIZ displays plan1..."
		rospy.sleep(5)

		print "============ Visualizing plan1"
		display_trajectory = moveit_msgs.msg.DisplayTrajectory()

		display_trajectory.trajectory_start = robot.get_current_state()
		display_trajectory.trajectory.append(plan1)
		display_trajectory_publisher.publish(display_trajectory);

		print "============ Waiting while plan1 is visualized (again)..."
		rospy.sleep(5)

		# move the arm
		#group.go(wait=True)
	

if __name__ == '__main__':
	mr = MoveRobot()
	mr.main()
