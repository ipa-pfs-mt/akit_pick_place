#include <akit_pick_place/akit_pick_place.h>

int main(int argc, char **argv){

  ros::init(argc, argv, "tool_exchange_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  akit_pick_place akit;
  akit.attachTool("gripper");
  sleep(2.0);
  akit.detachTool("gripper");
  sleep(2.0);
  akit.attachTool("bucket");
  sleep(2.0);
  akit.detachTool("bucket");
}
