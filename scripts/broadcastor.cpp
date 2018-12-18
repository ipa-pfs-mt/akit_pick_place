#include <akit_pick_place/akit_pick_place.h>

std::string BASE_LINK_FRAME = "root";

int main(int argc, char **argv){

  ros::init(argc, argv, "broadcaster");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  akit_pick_place panda;

  geometry_msgs::PoseStamped test_pose;
  test_pose.header.frame_id = BASE_LINK_FRAME;
  test_pose.pose.position.x = 0.75;
  test_pose.pose.position.y = 0.0;
  test_pose.pose.position.z = 0.1;
  test_pose.pose.orientation.w = 0.707;
  test_pose.pose.orientation.x = 0.0;
  test_pose.pose.orientation.y = 0.707;
  test_pose.pose.orientation.z = 0.0;

  panda.addCollisionBlock(test_pose.pose, "block", 0.05,0.05,0.15);

  panda.broadcastFrame(test_pose, "block");
}
