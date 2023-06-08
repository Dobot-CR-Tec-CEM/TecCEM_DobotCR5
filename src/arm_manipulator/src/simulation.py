#!/usr/bin/env python

import rospy, tf
import numpy as np
import random as rd
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import sys

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, PoseStamped


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python", anonymous=True)


robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "cr5_gripper_robot"
move_group = moveit_commander.MoveGroupCommander(group_name)


display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print ("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print ("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print ("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print ("============ Printing robot state")
print (robot.get_current_state())
print ("")

def wait_for_state_update(box_name, box_is_known=False, box_is_attached=False, timeout=0.5):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while(seconds - start < timeout) and not rospy.is_shutdown():
      attached_objetcs = scene.get_attached_objects([box_name])
      is_attached = len(attached_objetcs.keys()) > 0

      is_known = box_name in scene.get_known_object_names()
      if(box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False

# We can get the joint values from the group and adjust some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -np.pi/4
joint_goal[2] = 0
joint_goal[3] = -np.pi/2
joint_goal[4] = 0
joint_goal[5] = np.pi/3

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.4
pose_goal.position.y = 0.1
pose_goal.position.z = 0.4

move_group.set_pose_target(pose_goal)

plan = move_group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()

# We can get the joint values from the group and adjust some of the values:
joint_goal = move_group.get_current_joint_values()
# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)
# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()



pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = -1.0
pose_goal.position.x = 0.5
pose_goal.position.y = 0.5
pose_goal.position.z = 0.2

move_group.set_pose_target(pose_goal)

plan = move_group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()