#!/usr/bin/env python
import rospy
import sys
import tf2_ros
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import *
from math import pi

#from path_planner.srv import *

#rospy.wait_for_service("RequestGoal")
#request_handler = rospy.ServiceProxy("RequestGoal", RequestGoal)
#rospy.wait_for_service("AttachObject")
#attach_handler = rospy.ServiceProxy("AttachObject", AttachObject)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("cr5_gripper_node")

# ===== Global variables ===== #
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

box_name = ""
group_name = "cr5_gripper_robot"
gripper_group_name = "gripper"

move_group = moveit_commander.MoveGroupCommander(group_name)
gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)

eef_link = move_group.get_end_effector_link()
print(eef_link)

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

def add_table(nombre, x, y, z, size):
    table_name = nombre
    table_obs = PoseStamped()
    table_obs.header.frame_id = "dummy_link"
    table_obs.pose.orientation.w = 1.0
    table_obs.pose.position.x = x
    table_obs.pose.position.y = y
    table_obs.pose.position.z = z
    scene.add_box(table_name, table_obs, size=size)
    obstacle_added = wait_for_state_update(table_name, box_is_known=True, timeout=1.5)
    while not obstacle_added:
      scene.add_box(table_name, table_obs, size=size)
      obstacle_added = wait_for_state_update(table_name, box_is_known=True, timeout=1.5)




def getTf(goal_name):
    """
    Function that will get the name of
    the goal and will return the pose
    coordinate of that object
    """
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    pose = tf_buffer.lookup_transform("base_link", goal_name, rospy.Time(), rospy.Duration(1.0))
    return pose


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


def homePose():
    """
    Final home position after moving all
    colored cubes into the correspondant 
    deposits.
    """
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0
    move_group.get_named_target_values("home")
    move_group.go(joint_goal, wait=True)
    move_group.stop()    
    move_group.clear_pose_targets()
    

def go2Pose(x, y, z=0.30):
    move_group.set_planning_time(50.0);
    move_group.set_num_planning_attempts(50)
    #global move_group
    pose_goal = Pose()
    pose_goal.orientation.x = 1.0
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    move_group.set_pose_target(pose_goal, "Link6")
    plan = move_group.go(wait=True) #True en Gazebo
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets afte, r planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets().
    move_group.clear_pose_targets()

    #move_group.set_approximate_joint_value_target(pose_goal, "Link6")


def Down():
    pose_goal = move_group.get_current_pose().pose
    pose_goal.position.z -= 0.10
    move_group.set_pose_target(pose_goal, "Link6")
    plan = move_group.go(wait=True)
    

def Up():
    pose_goal = move_group.get_current_pose().pose
    pose_goal.position.z += 0.15
    move_group.set_pose_target(pose_goal, "Link6")
    plan = move_group.go(wait=True)

def attachBox(box_name):
    touch_links = robot.get_link_names(group=gripper_group_name)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)

def deattach(box_name):
    scene.remove_attached_object(eef_link, name=box_name)

def removeBox(box_name):
    scene.remove_world_object(box_name)

def main():
    """
    Main function that will excecute the
    instructions of the arm.
    """
    box_x = 0.1
    box_y = -0.3
    add_table("mesa", -0.02, -0.2, -0.09, (1.5, 0.801, 0.05))
    add_table("box1", box_x, box_y, -0.05, (0.05, 0.05, 0.05))
    
    homePose()
    #while not rospy.is_shutdown():
    print("Llendo a primer punto")
    go2Pose(box_x, box_y)
    print("Bajando")
    Down()
    #attach en rviz para que lo tome
    attachBox("box1")
    Up()
    print("Llendo a siguiente punto")
    go2Pose(-0.4, box_y)
    Down()
    # deattach en rviz para que lo tome
    deattach("box1")
    removeBox("box1")
    Up()
    homePose()
    rospy.signal_shutdown("Task Completed")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass