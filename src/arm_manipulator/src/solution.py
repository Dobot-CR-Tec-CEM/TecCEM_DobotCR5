#!/usr/bin/env python
import rospy
import sys
import tf2_ros
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import *
from math import pi

from path_planner.srv import *

rospy.wait_for_service("RequestGoal")
request_handler = rospy.ServiceProxy("RequestGoal", RequestGoal)
rospy.wait_for_service("AttachObject")
attach_handler = rospy.ServiceProxy("AttachObject", AttachObject)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("xarm6_node")

# ===== Global variables ===== #
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

box_name = ""
group_name = "xarm6"
gripper_group_name = "xarm_gripper"

move_group = moveit_commander.MoveGroupCommander(group_name)
gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)

eef_link = move_group.get_end_effector_link()
print(eef_link)
steps = ["pick" if x%2==0 else "place" for x in range(2)]

targets = ["RedBox",
           "BlueBox",
           "GreenBox"]

boxes = ["DepositBoxGreen",
         "DepositBoxRed",
         "DepositBoxBlue"]


def getCurrentState():
    print("==== Printing robot curent state ===")
    print(robot.get_current_state())
    touch_links = robot.get_link_names(group=gripper_group_name)
    print(touch_links)


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


def addObstacles():
    """
    Function that will add the obstacles
    into the space in Rviz for obstacle detection
    """
    global scene
    
    for i in boxes:
        box_pose = PoseStamped()
        box_pose.header.frame_id = i
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.01
        box_name = i
        scene.add_box(box_name, box_pose, size=(0.359288, 0.171428, 0.109964))
        obstacle_added = wait_for_state_update(box_name, box_is_known=True, timeout=1.5)
        while not obstacle_added:
            scene.add_box(box_name, box_pose, size=(0.359288, 0.171428, 0.109964))
            obstacle_added = wait_for_state_update(box_name, box_is_known=True, timeout=1.5)
        print("Obstacle ", i, " added: ", obstacle_added)

    for i in targets:
        box_pose = PoseStamped()
        box_pose.header.frame_id = i
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.01
        box_name = i
        scene.add_box(box_name, box_pose, size=(0.06, 0.06, 0.06))
        box_added = wait_for_state_update(box_name, box_is_known=True)
        while not box_added:
            scene.add_box(box_name, box_pose, size=(0.06, 0.06, 0.06))
            box_added = wait_for_state_update(box_name, box_is_known=True)
        print("Box ", i, " added: ", box_added)

    print("Objects added correctly")


def closeGripper(box_name):
    """
    Function that will close the gripper
    and call the attach service
    """
    global gripper_group
    attach_handler(True, box_name)
    gripper_group.set_joint_value_target([0.23,0.23,0.23,0.23,0.23,0.23])
    gripper_group.go(wait=True)
    gripper_group.stop()
    gripper_group.clear_pose_targets()


def openGripper(box_name):
    """
    Function that will open the gripper
    and call the attach service
    """
    global gripper_group
    gripper_group.set_named_target("open")
    gripper_group.go(wait=True)
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    attach_handler(False, box_name)


def getTf(goal_name):
    """
    Function that will get the name of
    the goal and will return the pose
    coordinate of that object
    """
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    pose = tf_buffer.lookup_transform("link_base", goal_name, rospy.Time(), rospy.Duration(1.0))
    return pose


def addBox(box_name):
    """
    Function that attach the box to the
    gripper in Rviz
    """
    global scene
    box_pose = PoseStamped()
    box_pose.header.frame_id = 'right_finger'
    box_pose.pose.orientation.w = 1.0
    #box_pose.pose.orientation.z = -1.0
    box_pose.pose.position.z = 0.04
    #box_pose.pose.position.x = -0.05
    box_pose.pose.position.y = 0.065
    box_name = box_name
    scene.add_box(box_name, box_pose, size=(0.06, 0.06, 0.06))
    obstacle_added = wait_for_state_update(box_name, box_is_known=True, timeout=1.5)
    while not obstacle_added:
        scene.add_box(box_name, box_pose, size=(0.06, 0.06, 0.06))
        obstacle_added = wait_for_state_update(box_name, box_is_known=True, timeout=1.5)
    
    touch_links = robot.get_link_names(group=gripper_group_name)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    added = wait_for_state_update(box_name, box_is_attached=True, box_is_known=True, timeout=1.5)
    print("Object linked")


def removeBox(box_name):
    """
    Function that will remove the
    attaced object from the Rviz
    interface
    """
    global scene
    scene.remove_attached_object(eef_link, name=box_name)
    deattached = wait_for_state_update(box_name, box_is_known=True, box_is_attached=False, timeout=1.5)
    while not deattached:
        scene.remove_attached_object(eef_link, name=box_name)
        deattached = wait_for_state_update(box_name, box_is_known=True, box_is_attached=False, timeout=1.5)
    scene.remove_world_object(box_name)
    removed = wait_for_state_update(box_name, box_is_attached=True, box_is_known=False, timeout=1.5)
    while not removed:
        scene.remove_world_object(box_name)
        removed = wait_for_state_update(box_name, box_is_attached=False, box_is_known=False, timeout=1.5)
    

def goToBox(box_name):
    """
    Function that receive the box name,
    gets the pose and make the movement
    of the arm using moveIt package.

    Deletes the box as obstacle and add
    it as an attached box into the gripper
    """
    global move_group, scene

    pose = getTf(box_name)

    pose_goal = Pose()
    pose_goal.orientation.x = 1.0
    pose_goal.position.x = pose.transform.translation.x
    pose_goal.position.y = pose.transform.translation.y
    pose_goal.position.z = pose.transform.translation.z + 0.10

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)

    move_group.stop()
    move_group.clear_pose_targets()

    scene.remove_world_object(box_name)
    deleted = wait_for_state_update(box_name, box_is_known=True, timeout=1.5)

    pose_goal.position.z -= 0.13
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    addBox(box_name)

    closeGripper(box_name)

    pose_goal.position.z += 0.10
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()


def goToDesposit(deposit_name, box_name):
    """
    Function that receive the deposit name,
    the box name and gets the pose. Makes the movement
    of the arm using moveIt package.

    Drops the object into the deposit by
    deataching the object and deleting it
    in the Rviz interface
    """
    global move_group, scene

    pose = getTf(deposit_name)

    pose_goal = Pose()
    pose_goal.orientation.x = 1.0
    pose_goal.position.x = pose.transform.translation.x
    pose_goal.position.y = pose.transform.translation.y
    pose_goal.position.z = pose.transform.translation.z + 0.10

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)

    move_group.stop()
    move_group.clear_pose_targets()
    
    openGripper(box_name)
    removeBox(box_name)


def homePose():
    """
    Final home position after moving all
    colored cubes into the correspondant 
    deposits.
    """
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    move_group.get_named_target_values("home")
    move_group.go(joint_goal, wait=True)
    move_group.stop()    
    move_group.clear_pose_targets()


def getGoal(action):
    """
    Function that calls the Request
    serive and gets the next goal
    where the arm has to go.
    """
    goal = request_handler(action)
    print(goal.status, goal.goal)
    if goal.status:
        return goal.goal
    else:
        return 'end'
    

def main():
    """
    Main function that will excecute the
    instructions of the arm.
    """
    global box_name
    action_index = 0
    #getCurrentState()
    addObstacles()
    action = steps[action_index % len(steps)]
    goal = getGoal(action)
    while(goal != 'End'):
        goToBox(goal)
        action_index += 1
        action = steps[action_index % len(steps)]
        box_name = goal
        goal = getGoal(action)
        goToDesposit(goal, box_name)
        action_index += 1
        action = steps[action_index % len(steps)]
        goal = getGoal(action)

    homePose()
    rospy.signal_shutdown("Task Completed")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass