/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cmath>
using namespace std;
// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;




void go_to_pose_goal(moveit::planning_interface::MoveGroupInterface &move_group_interface) {
  // .. _move_group_interface-planning-to-pose-goal:
  //
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // motion planning by controlling end-effector. 
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group_interface.setPoseTarget(target_pose1);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
  // Note that this can lead to problems if the robot moved in the meanwhile.
  move_group_interface.execute(my_plan);

}


void  go_to_joint_state 
(
  moveit::planning_interface::MoveGroupInterface &move_group_interface,
  const moveit::core::JointModelGroup* joint_model_group,
  moveit::core::RobotStatePtr &initial_pose
)

{

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  initial_pose->copyJointGroupPositions(joint_model_group, joint_group_positions);
  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  move_group_interface.setJointValueTarget(joint_group_positions);

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group_interface.execute(my_plan);
}

//Cartesian Path Planning
void draw_rectangle(
  moveit::planning_interface::MoveGroupInterface &move_group_interface) {

 // Cartesian Rectangle
  moveit_msgs::RobotTrajectory trajectory;
 std::vector<geometry_msgs::Pose> waypoints_rectangle;
 geometry_msgs::Pose start=move_group_interface.getCurrentPose().pose;

start.position.y-=0.1;
 waypoints_rectangle.push_back(start);
 start.position.x+=0.1;
 waypoints_rectangle.push_back(start);
 start.position.y+=0.1;
 waypoints_rectangle.push_back(start);
 start.position.x-=0.1;
 waypoints_rectangle.push_back(start);
 

double fraction = move_group_interface.computeCartesianPath(waypoints_rectangle, 0.01, 0.0, trajectory);
move_group_interface.execute(trajectory);
}


//Cartesian Path Planning
void draw_circle(moveit::planning_interface::MoveGroupInterface &move_group_interface) {

  //define center of circle
  double radius=0.1;
  double pi=3.14159;
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints_circle;
  geometry_msgs::Pose center=move_group_interface.getCurrentPose().pose;

  center.position.y-=0.1;

  geometry_msgs::Pose start=move_group_interface.getCurrentPose().pose;

  for (double angle=2*pi;angle>0;angle-=0.2) {
    start.position.x=center.position.x+radius*sin(angle);
    start.position.y=center.position.y+radius*cos(angle);
    waypoints_circle.push_back(start);
  }

  
  double fraction = move_group_interface.computeCartesianPath(waypoints_circle, 0.01, 0.0, trajectory);
  move_group_interface.execute(trajectory);
}


//Cartesian Path Planning
void draw_A(moveit::planning_interface::MoveGroupInterface &move_group_interface) {

moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints_A;
geometry_msgs::Pose start=move_group_interface.getCurrentPose().pose;

start.position.x-=0.1;
start.position.y-=0.1*sqrt(3);
waypoints_A.push_back(start);

start.position.x+=0.1;
start.position.y+=0.1*sqrt(3);
waypoints_A.push_back(start);

start.position.x+=0.1;
start.position.y-=0.1*sqrt(3);
waypoints_A.push_back(start);

start.position.x-=0.1;
start.position.y+=0.1*sqrt(3);
waypoints_A.push_back(start);

start.position.x-=0.05;
start.position.y-=0.05*sqrt(3);
waypoints_A.push_back(start);

start.position.x+=0.1;
waypoints_A.push_back(start);

  double fraction = move_group_interface.computeCartesianPath(waypoints_A, 0.01, 0.0, trajectory);
  move_group_interface.execute(trajectory);
}

void back_to_home(moveit::planning_interface::MoveGroupInterface &move_group_interface,
geometry_msgs::Pose &initial_pose) {

moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
geometry_msgs::Pose start=move_group_interface.getCurrentPose().pose;
waypoints.push_back(start);
waypoints.push_back(initial_pose);
double fraction = move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group_interface.execute(trajectory);

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();


  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "arm";

 // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


// Moving to a pose goal
 // motion planning by controlling angles of joints
  moveit::core::RobotStatePtr initial_pose = move_group_interface.getCurrentState();





// Let's get simple moving

go_to_pose_goal(move_group_interface);
go_to_joint_state(move_group_interface,joint_model_group,initial_pose);


draw_rectangle(move_group_interface);
go_to_joint_state(move_group_interface,joint_model_group,initial_pose);

draw_circle(move_group_interface);
go_to_joint_state(move_group_interface,joint_model_group,initial_pose);

draw_A(move_group_interface);
go_to_joint_state(move_group_interface,joint_model_group,initial_pose);

ros::shutdown();


  return 0;
}






