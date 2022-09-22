#include <math.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>


ros::Publisher pub_joint_traj_commands_;

void generate_traj()
{
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "whole_body_control");
  ros::NodeHandle* nh = new ros::NodeHandle();

  pub_joint_traj_commands_ =
    nh->advertise<trajectory_msgs::JointTrajectory>(
    "/darwin/full_body_traj_controller/command", 1000, true);

  //our goal variable
  trajectory_msgs::JointTrajectory goal;

  // First, the joint names, which apply to all waypoints
  goal.joint_names.push_back("neck_joint");
  goal.joint_names.push_back("l_shoulder_joint");
  goal.joint_names.push_back("r_shoulder_joint");
  goal.joint_names.push_back("l_biceps_joint");
  goal.joint_names.push_back("r_biceps_joint");
  goal.joint_names.push_back("l_elbow_joint");
  goal.joint_names.push_back("r_elbow_joint");
  goal.joint_names.push_back("l_hip_joint");
  goal.joint_names.push_back("r_hip_joint");
  goal.joint_names.push_back("l_thigh_joint");
  goal.joint_names.push_back("r_thigh_joint");
  goal.joint_names.push_back("l_knee_joint");
  goal.joint_names.push_back("r_knee_joint");
  goal.joint_names.push_back("l_ankle_joint");
  goal.joint_names.push_back("r_ankle_joint");
  goal.joint_names.push_back("l_foot_joint");
  goal.joint_names.push_back("r_foot_joint");

  // We will have two waypoints in this goal trajectory
  goal.points.resize(2);

  // First trajectory point
  // Positions
  int ind = 0;
  goal.points[ind].positions.resize(17);
  goal.points[ind].positions[0] = 4.0;
  goal.points[ind].positions[1] = 2.617994;
  goal.points[ind].positions[2] = 2.617994;
  goal.points[ind].positions[3] = 2.617994;
  goal.points[ind].positions[4] = 2.617994;
  goal.points[ind].positions[5] = 2.617994;
  goal.points[ind].positions[6] = 2.617994;
  goal.points[ind].positions[7] = 2.617994;
  goal.points[ind].positions[8] = 2.617994;
  goal.points[ind].positions[9] = 2.617994;
  goal.points[ind].positions[10] = 2.617994;
  goal.points[ind].positions[11] = 2.617994;
  goal.points[ind].positions[12] = 2.617994;
  goal.points[ind].positions[13] = 2.617994;
  goal.points[ind].positions[14] = 2.617994;
  goal.points[ind].positions[15] = 2.617994;
  goal.points[ind].positions[16] = 2.617994;
  // Velocities
  goal.points[ind].velocities.resize(17);
  for (size_t j = 0; j < 7; ++j)
  {
    goal.points[ind].velocities[j] = 0.0;
  }
  // To be reached 1 second after starting along the trajectory
  goal.points[ind].time_from_start = ros::Duration(1.0);

  // Second trajectory point
  // Positions
  ind += 1;
  goal.points[ind].positions.resize(17);
  goal.points[ind].positions[0] = 0.0;
  goal.points[ind].positions[1] = 2.617994;
  goal.points[ind].positions[2] = 2.617994;
  goal.points[ind].positions[3] = 2.617994;
  goal.points[ind].positions[4] = 2.617994;
  goal.points[ind].positions[5] = 2.617994;
  goal.points[ind].positions[6] = 2.617994;
  goal.points[ind].positions[7] = 2.617994;
  goal.points[ind].positions[8] = 2.617994;
  goal.points[ind].positions[9] = 2.617994;
  goal.points[ind].positions[10] = 2.617994;
  goal.points[ind].positions[11] = 2.617994;
  goal.points[ind].positions[12] = 2.617994;
  goal.points[ind].positions[13] = 2.617994;
  goal.points[ind].positions[14] = 2.617994;
  goal.points[ind].positions[15] = 2.617994;
  goal.points[ind].positions[16] = 2.617994;
  // Velocities
  goal.points[ind].velocities.resize(17);
  for (size_t j = 0; j < 7; ++j)
  {
    goal.points[ind].velocities[j] = 0.0;
  }
  // To be reached 2 seconds after starting along the trajectory
  goal.points[ind].time_from_start = ros::Duration(3.0);
  
  pub_joint_traj_commands_.publish(goal);

  ros::spin();

  return 0;
}

// /darwin/full_body_traj_controller/command
// /darwin/full_body_traj_controller/state
