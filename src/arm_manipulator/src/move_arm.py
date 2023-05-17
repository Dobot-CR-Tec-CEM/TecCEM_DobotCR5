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
from geometry_msgs.msg import Pose, PoseStamped, Quaternion


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("cr5_manipulator", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "cr5_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

rate = rospy.Rate(100)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory)

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


def go_home_pose():
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=False)
    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    rospy.sleep(30)


def move_joints():
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)
    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()


def go_to_goal(x=0, y=0, z=0, w=0):
    global robot
    q = Quaternion(*tf.transformations.quaternion_from_euler(np.pi, 0, 0))
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation = q
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    move_group.set_pose_target(pose_goal, "Link6")
    plan = move_group.plan()

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)

    move_group.go(wait=False)
    #move_group.clear_pose_targets()
    move_group.set_start_state_to_current_state()
    nueva_pose = move_group.get_current_state()
    print("JOINT VALUES", nueva_pose)

    move_group.execute(plan, wait=True)
    #rospy.loginfo("====>Moving to:\n{}".format(pose_goal))
    #sucess = move_group.move(wait=True)
    #print(sucess)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()

    rospy.sleep(20)

def get_current_pose():
    print(move_group.get_current_pose())

def main():
   while not rospy.is_shutdown():
      print("HOME")
    #   get_current_pose()
      go_home_pose()
      print("PUNTO 1")
      go_to_goal(0.5,0.5, 0.5)
      #move_joints()
      print("PUNTO 2")
      go_to_goal(-0.5,0.5, 0.5)
      print("PUNTO 3")
      go_to_goal(-0.5,-0.5, 0.5)
    #   rospy.sleep(20)


if __name__ == "__main__":
    try:
        main()
        # print("HOME")
        # go_home_pose()
        # print("PUNTO 1")
        # go_to_goal(0.5,0.5, 0.5)
        # print("PUNTO 2")
        # go_to_goal(-0.5,0.5, 0.5)
        # print("PUNTO 3")
        # go_to_goal(-0.5,-0.5, 0.5)
        # print("HOME")
        # go_home_pose()
    except rospy.ROSInterruptException:
       print("Stopping node")
       exit()
exit()

