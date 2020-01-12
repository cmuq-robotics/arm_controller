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


#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/PlanningScene.h>




int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_interface");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("Waiting for context to come up");
  sleep(10);
  ROS_INFO("Starting interface...");


  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.




  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  // visual_tools.loadRemoteControl();

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
/* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  std::cout << "EEF Name: " << joint_model_group->getEndEffectorName() << std::endl;

  // const robot_state::JointModelGroup* joint_model_group =
  //    move_group.getCurrentState()->getJointModelGroup("arm");

  // joint_model_group->printGroupInfo();
  move_group.setNumPlanningAttempts(10);


  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  // ROS_INFO_NAMED("tutorial", "End effector link: %s", joint_model_group->getEndEffectorName().c_str());


  geometry_msgs::PoseStamped robot_pose;
  robot_pose = move_group.getCurrentPose();



  geometry_msgs::Pose current_position;
  current_position = robot_pose.pose;

  /*Retrive position and orientation */
  geometry_msgs::Point exact_pose; 
  geometry_msgs::Quaternion exact_orientation; 
  exact_pose = current_position.position;
  exact_orientation = current_position.orientation;

  // while(1){
  //   robot_pose = move_group.getCurrentPose();
  //   current_position = robot_pose.pose;
  //   exact_pose = current_position.position;
  //   exact_orientation = current_position.orientation;
    
  //   std::cout<<"Reference frame: "<<robot_pose.header.frame_id<<std::endl;
  //   ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
  //   std::cout<<"Robot position : "<<exact_pose.x<<"\t"<<exact_pose.y<<"\t"<<exact_pose.z<<std::endl;
  //   std::cout<<"Robot Orientation : "<<exact_orientation.x<<"\t"<<exact_orientation.y<<"\t"<<exact_orientation.z<<"\t"<<exact_orientation.w<< "\n" << std::endl;
  //   sleep(3);
  // }
  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^

  move_group.getPlanningFrame();
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  geometry_msgs::Pose target_pose1;

  target_pose1.position.x = -0.0237;
  target_pose1.position.y = 0.0897;
  target_pose1.position.z = 0.3766;
  target_pose1.orientation.w = 1;
  // target_pose1.orientation.x = exact_orientation.x;
  // target_pose1.orientation.y = exact_orientation.y;
  // target_pose1.orientation.z = exact_orientation.z;

  move_group.setPoseTarget(target_pose1);


  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");

  // // Now, we call the planner to compute the plan and visualize it.
  // // Note that we are just planning, not asking move_group
  // // to actually move the robot.
  // 

  // bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  // sleep(5);
  // move_group.execute(plan);

  return 0;
}
