#!/usr/bin/env python

import rospy, tf
import numpy as np
import tf2_ros
import random as rd
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import sys

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from dobot_bringup.srv import *

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("cr5_node", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "cr5_gripper_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

rate = rospy.Rate(10)

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
    joint_goal[0] = np.pi/2
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
    rospy.sleep(10)


def move_joints():
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -np.pi/2
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
    rospy.sleep(14)


def addObstacles():
    """
    Function that will add the obstacles
    into the space in Rviz for obstacle detection
    """
    global scene
    
    for i in ["box1"]:
        box_pose = PoseStamped()
        box_pose.header.frame_id = robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.01
        box_name = i
        scene.add_box(box_name, box_pose, size=(0.359288, 0.171428, 0.109964))
        obstacle_added = wait_for_state_update(box_name, box_is_known=True, timeout=1.5)
        while not obstacle_added:
            scene.add_box(box_name, box_pose, size=(0.359288, 0.171428, 0.109964))
            obstacle_added = wait_for_state_update(box_name, box_is_known=True, timeout=1.5)
        print("Obstacle ", i, " added: ", obstacle_added)
  

def speedFactor():
    rospy.wait_for_service("/dobot_bringup/srv/SpeedFactor")
    
    try:
        speed = rospy.ServiceProxy("/dobot_bringup/srv/SpeedFactor", SpeedFactor)
        resp = speed(30)
        return resp
    except rospy.ServiceException as e:
        print("Service Failed:", e)



def getTf(goal_name):
    """
    Function that will get the name of
    the goal and will return the pose
    coordinate of that object
    """
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    pose = tf_buffer.lookup_transform("dummy_link", goal_name, rospy.Time(), rospy.Duration(1.0))
    return pose


def go_to_goal(x=0, y=0, z=0, w=0):
    global robot
    q = Quaternion(*tf.transformations.quaternion_from_euler(np.pi, 0, 0))
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation = q
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    move_group.set_pose_target(pose_goal)
    plan = move_group.plan()

    
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    #move_group.set_pose_target(move_group.get_current_pose())
    display_trajectory.trajectory.append(plan)


    display_trajectory_publisher.publish(display_trajectory)

    move_group.go(wait=False)
    #move_group.set_start_state_to_current_state()
    #nueva_pose = move_group.get_current_state()
    #print("JOINT VALUES", nueva_pose)

    #move_group.execute(plan, wait=True)
    move_group.clear_pose_targets()
    #rospy.loginfo("====>Moving to:\n{}".format(pose_goal))
    #sucess = move_group.move(wait=True)
    # We can get the joint values from the group and adjust some of the values:
    #joint_goal = move_group.get_current_joint_values()
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    #move_group.go(joint_goal, wait=True)
    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    print("TERMIANDO DE MOVERSE")
    # Calling `stop()` ensures that there is no residual movement
    #move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()

    rospy.sleep(14)

def get_current_pose():
    print(move_group.get_current_pose())

def add_table():
    table_name = "mesa"
    table_obs = PoseStamped()
    table_obs.header.frame_id = "dummy_link"
    table_obs.pose.orientation.w = 1.0
    table_obs.pose.position.z = -0.05
    table_obs.pose.position.x = -0.02
    table_obs.pose.position.y = -0.2
    scene.add_box(table_name, table_obs, size=(1.5, 0.801, 0.1))
    obstacle_added = wait_for_state_update(table_name, box_is_known=True, timeout=1.5)
    while not obstacle_added:
        scene.add_box(table_name, table_obs, size=(1.5, 0.801, 0.1))
    obstacle_added = wait_for_state_update(table_name, box_is_known=True, timeout=1.5)

def armOrientation():
    rospy.wait_for_service("/dobot_bringup/srv/SetArmOrientation")
    
    try:
        arm = rospy.ServiceProxy("/dobot_bringup/srv/SetArmOrientation", SetArmOrientation)
        resp = arm(-1, 1, 1, 1)
        return resp
    except rospy.ServiceException as e:
        print("Service Failed:", e)


def main():
   speedFactor()
   armOrientation()
   add_table()
   while not rospy.is_shutdown():
      print("HOME")
    #   get_current_pose()
      go_home_pose()
      #move_joints()
      #print(getTf("box1"))
      print("PUNTO 1")
      armOrientation()
      go_to_goal(-0.593, -0.318, 0.55)
      print("Bajando")
      armOrientation()
      go_to_goal(-0.593, -0.318, 0.32)
      print("Subiendo")
      armOrientation()
      go_to_goal(-0.593, -0.318, 0.55)
      #exit()
      print("PUNTO 2")
      armOrientation()
      go_to_goal(0.433, -0.285, 0.55)
      print("Bajando")
      armOrientation()
      go_to_goal(0.433, -0.285, 0.32)
      print("Subiendo")
      armOrientation()
      go_to_goal(0.433, -0.285, 0.55)
      #print("PUNTO 3")
      #go_to_goal(-0.3,-0.5, 0.35)
        
      #rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
       print("Stopping node")
       exit()
exit()
