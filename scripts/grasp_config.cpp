#include <akit_pick_place/akit_pick_place.h>

std::string BASE_LINK_FRAME = "root";
std::string OBJECT_FRAME = "block";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_config");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  akit_pick_place akit;
  tf::TransformListener transform_listener;

  moveit::planning_interface::MoveGroupInterface move_group("gripper");

  geometry_msgs::PoseStamped eef_parent_link_pose = move_group.getCurrentPose("j2n6s300_end_effector");
  ROS_INFO_STREAM("Frame " << eef_parent_link_pose.header.frame_id);
  ROS_INFO_STREAM("x: " << eef_parent_link_pose.pose.position.x);
  ROS_INFO_STREAM("y: " << eef_parent_link_pose.pose.position.y);
  ROS_INFO_STREAM("z: " << eef_parent_link_pose.pose.position.z);
  ROS_INFO_STREAM("orientation x: " << eef_parent_link_pose.pose.orientation.x);
  ROS_INFO_STREAM("orientation y: " << eef_parent_link_pose.pose.orientation.y);
  ROS_INFO_STREAM("orientation z: " << eef_parent_link_pose.pose.orientation.z);
  ROS_INFO_STREAM("orientation w: " << eef_parent_link_pose.pose.orientation.w);

  geometry_msgs::PoseStamped block_frame;

  ROS_INFO_STREAM("****************************************************************");

  // transform object from world frame to gripper rotator frame, wait to avoid time difference exceptions
  transform_listener.waitForTransform(OBJECT_FRAME, BASE_LINK_FRAME, ros::Time::now(), ros::Duration(0.1));
  transform_listener.transformPose(OBJECT_FRAME, ros::Time(0), eef_parent_link_pose, BASE_LINK_FRAME, block_frame);

  ROS_INFO_STREAM("Frame " << block_frame.header.frame_id);
  ROS_INFO_STREAM("x: " << block_frame.pose.position.x);
  ROS_INFO_STREAM("y: " << block_frame.pose.position.y);
  ROS_INFO_STREAM("z: " << block_frame.pose.position.z);
  ROS_INFO_STREAM("orientation x: " << block_frame.pose.orientation.x);
  ROS_INFO_STREAM("orientation y: " << block_frame.pose.orientation.y);
  ROS_INFO_STREAM("orientation z: " << block_frame.pose.orientation.z);
  ROS_INFO_STREAM("orientation w: " << block_frame.pose.orientation.w);

  std::vector<geometry_msgs::PoseStamped> grasps = akit.generateGrasps("block");
  akit.visualizeGraspPose(grasps);
}
