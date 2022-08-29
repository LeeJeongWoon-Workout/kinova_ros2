#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()


        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )


        #Create gripper group module
        gripper_group_name="gripper"
        gripper_group=moveit_commander.MoveGroupCommander(gripper_group_name)
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        print("============ Printing gripper joint name")
        print(gripper_joint_names[0])
        print("")


        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.gripper_group=gripper_group
        self.gripper_joint_name=gripper_joint_names[0]


    #Planning by controlling each joint 
    def go_to_joint_state(self):


        move_group = self.move_group

        ## Planning to a Joint Goal
        ## Kinova Gen 3 Lite has 6 joints
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()


        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    #Planning by controlling pose of end-effector
    def go_to_pose_goal(self):

        move_group = self.move_group

        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        #we set pose target of end-effector
        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()


        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


    """
    Cartesian planning is the generation of motion trajectories where a goal is specified in terms of the
    desired location of the end effector.
    """
    def plan_cartesian_path(self, scale=1):

        move_group = self.move_group

        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction


    def display_trajectory(self, plan):

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher


        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)


    def execute_plan(self, plan):


        move_group = self.move_group


        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

    # Gripper handling 
    def reach_gripper_position(self,relative_position):
        gripper_group=self.gripper_group

        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            return val
        except:
            return False 




    def reach_named_position(self,target):
        # home or vertical
        arm_group=self.move_group

        #Going to one of those targets
        rospy.loginfo("Going to named target" + target)

        #set the target
        arm_group.set_named_target(target)
        #plan the trajectory
        planned_path=arm_group.plan()
        #Execute the trajectory and block while it's not finished
        arm_group.execute(planned_path,wait=True)


    def get_cartesian_pose(self):
        arm_group=self.move_group

        #Get the current pose
        pose=arm_group.get_current_pose()

        return pose.pose


    """
    Cartesian planning is the generation of motion trajectories where a goal is specified in terms of the
    desired location of the end effector.
    """
    def compute_cartesian_path(self,waypoints):
        
    
        arm_group=self.move_group
        (plan,fraction)=arm_group.compute_cartesian_path(waypoints,0.01,0.0)
        arm_group.execute(plan,wait=True)
        rospy.sleep(3)


    def draw_circle(self):
            rospy.loginfo("Circle")
            waypoints = []

            # start with the current pose
            start_pose=self.get_cartesian_pose()
            start_pose.orientation.w=1.0

            for i in range(36):
              wpose=copy.deepcopy(start_pose)
              wpose.position.x=start_pose.position.x+0.1*math.cos(i*0.2)
              wpose.position.y=start_pose.position.y+0.1*math.sin(i*0.2)
              waypoints.append(copy.deepcopy(wpose))

            self.compute_cartesian_path(waypoints)
            self.reach_named_position("home")
      
    def draw_rectangle(self):

            rospy.loginfo("Rectangle")
            waypoints=[]

            #start with the current pose
            start_pose=self.get_cartesian_pose()
            start_pose.orientation.w=1.0

            wpose=copy.deepcopy(start_pose)
            wpose.position.x-=0.2
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.y-=0.2
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x+=0.2
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.y+=0.2
            waypoints.append(copy.deepcopy(wpose))

            self.compute_cartesian_path(waypoints)
            self.reach_named_position("home")

    def draw_triangle(self):
            #Triangle
            rospy.loginfo("Triangle")
            waypoints=[]

            #start with the current pose
            start_pose=self.get_cartesian_pose()
            start_pose.orientation.w=1.0

            wpose=copy.deepcopy(start_pose)
            wpose.position.x-=0.1
            wpose.position.y-=0.1*math.sqrt(3)
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x+=0.2
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x-=0.1
            wpose.position.y+=0.1*math.sqrt(3)
            waypoints.append(copy.deepcopy(wpose))

            self.compute_cartesian_path(waypoints)
            self.reach_named_position("home")

    def draw_a(self):
            #Draw A
            #start with the current pose
            rospy.loginfo("Draw Alphabet A")
            waypoints=[]
            center_pose=self.get_cartesian_pose()
            center_pose.orientation.w=1.0

            wpose=copy.deepcopy(center_pose)
            wpose.position.x-=0.1
            wpose.position.y-=0.1*math.sqrt(3)
            waypoints.append(copy.deepcopy(wpose))


            wpose.position.x+=0.1
            wpose.position.y+=0.1*math.sqrt(3)
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x+=0.1
            wpose.position.y-=0.1*math.sqrt(3)
            waypoints.append(copy.deepcopy(wpose))


            wpose.position.x-=0.1
            wpose.position.y+=0.1*math.sqrt(3)
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x-=0.05
            wpose.position.y-=0.05*math.sqrt(3)
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x+=0.1
            waypoints.append(copy.deepcopy(wpose))
            self.compute_cartesian_path(waypoints)
            self.reach_named_position("home")




def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        tutorial = MoveGroupPythonInterfaceTutorial()

        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
        tutorial.go_to_joint_state()

        input("============ Press `Enter` to execute a movement using a pose goal ...")
        tutorial.go_to_pose_goal()

        input("============ Press `Enter` to plan and display a Cartesian path ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path()

        input(
            "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        )
        tutorial.display_trajectory(cartesian_plan)

        input("============ Press `Enter` to execute a saved path ...")
        tutorial.execute_plan(cartesian_plan)


        input("============ Press `Enter` to go to home ...")
        tutorial.reach_named_position("home")

        input("============ Press 'Enter' to draw Circle ...")
        tutorial.draw_circle()

        input("============ Press 'Enter' to draw Rectangle ...")
        tutorial.draw_rectangle()

        input("============ Press 'Enter' to draw triangle ...")
        tutorial.draw_triangle()


        input("============ Press 'Enter' to draw A ...")
        tutorial.draw_a()

        input("============ Press 'Enter' to open half ...")
        tutorial.reach_gripper_position(0.5)



        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
