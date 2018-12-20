#include <akit_pick_place/akit_pick_place.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_pick_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  akit_pick_place akit;

  // place location vector
  std::vector<geometry_msgs::Pose> place_locations_;
  geometry_msgs::Pose placeLocation1;
  placeLocation1.position.z = 0.17;
  placeLocation1.position.x = 2.0;
  placeLocation1.position.y = 3.0;
  placeLocation1.orientation.x = placeLocation1.orientation.y = placeLocation1.orientation.z = 0.0;
  placeLocation1.orientation.w = 1.0;
  geometry_msgs::Pose placeLocation2 = placeLocation1;
  placeLocation2.position.x = 1.0;
  geometry_msgs::Pose placeLocation3 = placeLocation1;
  placeLocation3.position.x = 0.0;
  geometry_msgs::Pose placeLocation4 = placeLocation1;
  placeLocation4.position.x = -1.0;

  place_locations_.push_back(placeLocation1);
  place_locations_.push_back(placeLocation2);
  place_locations_.push_back(placeLocation3);
  place_locations_.push_back(placeLocation4);

  akit.interactive_pick_place(place_locations_);
}
