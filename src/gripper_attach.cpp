#include <akit_pick_place/akit_pick_place.h>

int main(int argc, char **argv){

  ros::init(argc, argv, "gripper_attach");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /*robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);*/

  moveit::planning_interface::MoveGroupInterface akitGroup("e1_complete");
  moveit::planning_interface::MoveGroupInterface::Plan motionPlan;
  tf::TransformListener listener;

  /*collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;
  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  acm.setEntry("quickcoupler","gripper_rotator", true);
  acm.setEntry("stick","gripper_rotator",true);
  collision_result.clear();
  planning_scene.getCurrentState();*/

  geometry_msgs::PoseStamped marker_pose;
  marker_pose.header.frame_id = "gripper_rotator";
  marker_pose.pose.position.x = -0.1;
  marker_pose.pose.position.y = 0.0;
  marker_pose.pose.position.z = 0.4;
  marker_pose.pose.orientation.x = 0.0;
  marker_pose.pose.orientation.y = -0.707;
  marker_pose.pose.orientation.z = 0.0;
  marker_pose.pose.orientation.w = 0.707;

  geometry_msgs::PoseStamped marker_pose_in_world_frame;
  //transform object pose from gripper frame to chassis frame
  listener.waitForTransform("chassis","gripper_rotator", ros::Time::now(), ros::Duration(0.1)); //avoid time difference exception
  listener.transformPose("chassis",ros::Time(0), marker_pose, "gripper_rotator", marker_pose_in_world_frame);

  ros::Publisher markerP;
  markerP = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  uint32_t shape = visualization_msgs::Marker::ARROW;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "gripper_rotator";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.type = shape;
  marker.scale.x = 0.15;
  marker.scale.y = 0.025;
  marker.scale.z = 0.025;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = marker_pose.pose;

  while (markerP.getNumSubscribers() < 1)
      {
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
      }
   markerP.publish(marker);

   akitGroup.setPoseTarget(marker_pose_in_world_frame.pose);
   bool success = (akitGroup.plan(motionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   if(success)
     akitGroup.execute(motionPlan);
}




