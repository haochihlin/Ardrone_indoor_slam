#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/String.h"
#include <moveit_msgs/DisplayTrajectory.h> 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "replan2");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  //TEST MOVE.
  moveit::planning_interface::MoveGroup group("altHold");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
moveit_msgs::DisplayTrajectory display_trajectory;


ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

std::vector<double> group_variable_values;
group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

group_variable_values[0] = 2.0;
group_variable_values[1] = 3.0;
group.setJointValueTarget(group_variable_values);

  moveit::planning_interface::MoveGroup::Plan my_plan;
bool success = group.plan(my_plan);

ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
/* Sleep to give Rviz time to visualize the plan. */
sleep(5.0);




if (1)
{
  ROS_INFO("Visualizing plan 1 (again)");
  display_trajectory.trajectory_start = my_plan.start_state_;
  display_trajectory.trajectory.push_back(my_plan.trajectory_);
  display_publisher.publish(display_trajectory);
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);
}
  
  return 0;
}

