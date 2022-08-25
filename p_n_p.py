#!/usr/bin/env python


import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import math
from std_srvs.srv import Empty
import copy
from moveit_commander.conversions import pose_to_list, list_to_pose


def wait_for_state_update(scene, box_name, box_is_known=False, box_is_attached=False, timeout=4):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        rospy.sleep(0.1)
        seconds = rospy.get_time()
    return False


def main():

    #Initialize the node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    #Create the MoveItInterface necessary objects
    arm_group_name="arm"
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    #Create arm group module
    arm_group=moveit_commander.MoveGroupCommander(arm_group_name)
    display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

    #Create gripper group module
    gripper_group_name="gripper"
    gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)
    gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
    gripper_joint_name = gripper_joint_names[0]


    #Setting Environment
    table_0 = geometry_msgs.msg.PoseStamped()
    table_0.header.frame_id = arm_group.get_planning_frame()
    table_0.pose.position.x =0.5
    table_0.pose.position.y =0
    table_0.pose.position.z =0.05
    table_0.pose.orientation.w = 1.0   

    scene.add_box('table_0', table_0, size=(0.5, 0.5, 0.1))
    wait_for_state_update(scene=scene,box_name='table_0',box_is_known=True)

    cube_0 = geometry_msgs.msg.PoseStamped()
    cube_0.header.frame_id = arm_group.get_planning_frame()
    cube_0.pose.position.x =0.35
    cube_0.pose.position.y =0
    cube_0.pose.position.z =0.13
    cube_0.pose.orientation.w = 1.0                            

    scene.add_box('cube_0', cube_0,size=(0.05, 0.05, 0.05))
    wait_for_state_update(scene=scene,box_name='cube_0',box_is_known=True)

    cube_1 = geometry_msgs.msg.PoseStamped()
    cube_1.header.frame_id = arm_group.get_planning_frame()
    cube_1.pose.position.x =0.35
    cube_1.pose.position.y =0
    cube_1.pose.position.z =0.18
    cube_1.pose.orientation.w = 1.0                            

    scene.add_box('cube_1', cube_1,size=(0.05, 0.05, 0.05))
    wait_for_state_update(scene=scene,box_name='cube_1',box_is_known=True)

    # move to pose goal
    rospy.loginfo("Going to home")
    arm_group.set_named_target("home")
    plan=arm_group.plan()
    arm_group.execute(plan,wait=True)


    waypoints=[]
    pose_goal=list_to_pose([0.35, 0, 0.28, 0, -pi, -pi/2])
    waypoints.append(pose_goal)

    plan, _ = arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold
    arm_group.execute(plan,wait=True)



    # Gripper Open    
    # We only have to move this joint because all others are mimic!
    gripper_joint = robot.get_joint(gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    gripper_joint.move(0.5 * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)

    #list_to_pose function transforms 6DOF input to pose data for calculating path.
    #Move for grasping cube0
    waypoints=[]
    pose_goal=list_to_pose([0.35, 0, 0.20, 0, -pi, -pi/2])
    waypoints.append(pose_goal)

    plan, _ = arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold
    arm_group.execute(plan,wait=True)
    
    #Grasping
    touch_links=['left_finger_dist_link','left_finger_dist_link']
    gripper_group.attach_object('cube_1','gripper_base_link',touch_links=touch_links)

    #Move for detaching
    waypoints=[]
    pose_goal1=list_to_pose([0.35, 0, 0.23, 0, -pi, -pi/2])
    pose_goal2=list_to_pose([0.35, 0.2, 0.23, 0, -pi, -pi/2])
    pose_goal3=list_to_pose([0.35, 0.2, 0.13, 0, -pi, -pi/2])
    waypoints.append(pose_goal1)
    waypoints.append(pose_goal2)
    waypoints.append(pose_goal3)
    plan, _ = arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold
    arm_group.execute(plan,wait=True)


    #Detach Object
    gripper_group.detach_object('cube_1')


    rospy.loginfo("Going to home")
    arm_group.set_named_target("home")
    plan=arm_group.plan()
    arm_group.execute(plan,wait=True)

    #Move for grasping cube1
    waypoints=[]
    pose_goal=list_to_pose([0.35, 0, 0.28, 0, -pi, -pi/2])
    waypoints.append(pose_goal)

    plan, _ = arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold
    arm_group.execute(plan,wait=True)

    waypoints=[]
    pose_goal=list_to_pose([0.35, 0, 0.15, 0, -pi, -pi/2])
    waypoints.append(pose_goal)

    plan, _ = arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold
    arm_group.execute(plan,wait=True)

    #Grasping
    touch_links=['left_finger_dist_link','left_finger_dist_link']
    gripper_group.attach_object('cube_0','gripper_base_link',touch_links=touch_links)

    #Move for detaching
    waypoints=[]
    pose_goal1=list_to_pose([0.35, 0, 0.28, 0, -pi, -pi/2])
    pose_goal2=list_to_pose([0.35, 0.2, 0.28, 0, -pi, -pi/2])
    pose_goal3=list_to_pose([0.35, 0.2, 0.18, 0, -pi, -pi/2])
    waypoints.append(pose_goal1)
    waypoints.append(pose_goal2)
    waypoints.append(pose_goal3)
    plan, _ = arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold
    arm_group.execute(plan,wait=True)


    #Detach Object
    gripper_group.detach_object('cube_0')

    rospy.loginfo("Going to home")
    arm_group.set_named_target("home")
    plan=arm_group.plan()
    arm_group.execute(plan,wait=True)







if __name__=='__main__':
  main()