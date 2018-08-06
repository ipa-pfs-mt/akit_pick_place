#include <akit_pick_place/akit_pick_place.h>

int main(int argc, char **argv){

  ros::init(argc, argv, "tool_exchange_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  akit_pick_place akit;
  akit.attachTool("gripper_rotator");
  sleep(1.0);
  akit.detachTool("gripper_rotator");
  sleep(1.0);
  akit.attachTool("bucket_raedlinger");
  sleep(1.0);
  akit.detachTool("bucket_raedlinger");
}
