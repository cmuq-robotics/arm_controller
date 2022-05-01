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
#include "arm_interface/pick.h"
#include "arm_interface/place.h"

#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/PlanningScene.h>



void gotopose( robot_state::RobotStatePtr robot_state,
	       moveit::planning_interface::MoveGroupInterface &move_group,
	       const robot_state::JointModelGroup *joint_model_group,
	       geometry_msgs::Pose &target_pose)
{
  ROS_INFO("Go to Pose");

  std::vector<double> joint_values;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
    
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  ROS_INFO("Trying IK.....");
  if(robot_state->setFromIK(joint_model_group, target_pose)){
    ROS_INFO("successfully retrieved IK Solution!");
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      if( joint_names[i] == "wrist_roll_joint")
	{
	  joint_values[i] =0;
	}
      move_group.setJointValueTarget(joint_values);
    }
  }
  else{
    ROS_ERROR("CANNOT SOLVE IK");
  }
  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");

  sleep(1);
  move_group.execute(plan);

}


bool set_gripper(std::string conf)
{
  ROS_INFO("Setting gripper to %s", conf.c_str());
  static const std::string PLANNING_GROUP = "gripper";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  move_group.setJointValueTarget(move_group.getNamedTargetValues(conf));


  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan computation done  %s",success?"":"FAILED");

  //move_group.execute(plan);
  move_group.move();
  sleep(1);
  ROS_INFO("Gripper conf done");
  
}

bool pick(arm_interface::pick::Request  &req,
	  arm_interface::pick::Request  &response)
{
  ROS_INFO("Picking Object");
  static const std::string PLANNING_GROUP = "arm";
  geometry_msgs::Pose target_pose = req.target;
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group
    = robot_state->getJointModelGroup(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  std::vector<double> joint_values;
  std::cout << "EEF Name: " << joint_model_group->getEndEffectorName() << std::endl;

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  move_group.setJointValueTarget(move_group.getNamedTargetValues("right_up"));

  ROS_INFO("Setting initial arm configuration");
  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan computation done  %s",success?"":"FAILED");

  //move_group.execute(plan);
  move_group.move();
  sleep(1);
  ROS_INFO("Pre-grasp configuration");
  target_pose.position.z += 0.10;
  gotopose(robot_state, move_group, joint_model_group, target_pose);
  sleep(1);
  set_gripper("grip_open");
  target_pose.position.z -= 0.07;
  gotopose(robot_state, move_group, joint_model_group, target_pose);
  sleep(1);
  set_gripper("grip_closed");
  sleep(1);
  move_group.setJointValueTarget(move_group.getNamedTargetValues("right_up"));
  ROS_INFO("Setting final arm configuration");
  success &= (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan computation done  %s",success?"":"FAILED");
  move_group.move();
  sleep(1);
  ROS_INFO("Grasp done!");
  return success;
  
}

bool place(arm_interface::pick::Request  &req,
	  arm_interface::pick::Request  &response)
{
  ROS_INFO("Placing Object");
  static const std::string PLANNING_GROUP = "arm";
  geometry_msgs::Pose target_pose = req.target;
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group
    = robot_state->getJointModelGroup(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  std::vector<double> joint_values;
  std::cout << "EEF Name: " << joint_model_group->getEndEffectorName() << std::endl;

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  move_group.setJointValueTarget(move_group.getNamedTargetValues("right_up"));

  ROS_INFO("Setting initial arm configuration");
  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan computation done  %s",success?"":"FAILED");

  //move_group.execute(plan);
  move_group.move();
  sleep(1);
  ROS_INFO("Pre-place configuration");
  target_pose.position.z += 0.10;
  gotopose(robot_state, move_group, joint_model_group, target_pose);
  sleep(1);
  target_pose.position.z -= 0.07;
  gotopose(robot_state, move_group, joint_model_group, target_pose);
  sleep(1);
  set_gripper("grip_open");
  sleep(1);
  move_group.setJointValueTarget(move_group.getNamedTargetValues("right_up"));
  ROS_INFO("Setting final arm configuration");
  success &= (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan computation done  %s",success?"":"FAILED");
  move_group.move();
  sleep(1);
  ROS_INFO("Place done!");
  return success;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_interface");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ROS_INFO("Waiting for context to come up");
  sleep(10);
  ROS_INFO("Starting interface...");

  
  ros::ServiceServer servicepick = node_handle.advertiseService("pick", pick);
  ros::ServiceServer serviceplace = node_handle.advertiseService("place", place);
  ROS_INFO("Ready to pick & place objects");

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
  

  /*   Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/

  // const robot_state::JointModelGroup* joint_model_group =
  //    move_group.getCurrentState()->getJointModelGroup("arm");

  // joint_model_group->printGroupInfo();
  // move_group.setNumPlanningAttempts(10);
  // const double* elbow_pos = robot_state->getJointPositions("elbow_pitch_joint");
  // std::cout << "JOINT POS OF ELBOW PITCH: " << &elbow_pos << std::endl;
  // robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  // for (std::size_t i = 0; i < joint_names.size(); ++i)
  // {
  //   ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  // }

  //ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  //ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  // ROS_INFO_NAMED("tutorial", "End effector link: %s", joint_model_group->getEndEffectorName().c_str());




  /*Retrive position and orientation */
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






  geometry_msgs::Pose target_pose1;

  target_pose1.position.x = 0.25;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 0.05;

  target_pose1.orientation.x = 0.0000000;
  target_pose1.orientation.y = 0.000000;
  target_pose1.orientation.z = 0.00000;
  target_pose1.orientation.w = 1;

  //  pick(target_pose1);


  //  ros::spin();
  ros::waitForShutdown();

  return 0;
}
