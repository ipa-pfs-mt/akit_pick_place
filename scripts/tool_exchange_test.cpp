#include <akit_pick_place/akit_pick_place.h>

int main(int argc, char **argv){

  ros::init(argc, argv, "tool_exchange_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  akit_pick_place akit;
  akit.attachTool("gripper");
}
