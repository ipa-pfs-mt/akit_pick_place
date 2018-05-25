#include <akit_pick_place/akit_pick_place.h>

double quickcoupler_z = 0.13;  //distance between quickcoupler frame origin and lock in z-direction
double quickcoupler_x = 0.035; //distance between quickcoupler frame origin and edge in x-direction

int main(int argc, char **argv){

  ros::init(argc, argv, "gripper_attach");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  akit_pick_place akit;
  akit.attachGripper();

  ros::ServiceClient plannnig_scene_diff_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  moveit::planning_interface::MoveGroupInterface akitGroup("e1_complete");
  moveit::planning_interface::MoveGroupInterface::Plan motionPlan;
  tf::TransformListener listener;

  const robot_state::JointModelGroup *joint_model_group = akitGroup.getCurrentState()->getJointModelGroup("e1_complete");

  moveit_msgs::PlanningScene planning_scene_msg;
  moveit_msgs::ApplyPlanningScene planning_scene_srv;

  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  acm.setEntry("quickcoupler","gripper_rotator", true);
  acm.setEntry("stick","gripper_rotator",true);

  acm.getMessage(planning_scene_msg.allowed_collision_matrix);
  planning_scene_msg.is_diff = true;
  planning_scene_srv.request.scene = planning_scene_msg;
  plannnig_scene_diff_client.call(planning_scene_srv);

  tf::Quaternion q = tf::createQuaternionFromRPY(0.0,-M_PI/2,0.0);
  geometry_msgs::PoseStamped marker_pose;
  marker_pose.header.frame_id = "gripper_rotator";
  marker_pose.pose.position.x = - quickcoupler_z; //translate the quickcoupler so that both locks match
  marker_pose.pose.position.y = 0.0;
  marker_pose.pose.position.z = 0.25; //adjust
  marker_pose.pose.orientation.w = q[3]; //rotation 90d around y
  marker_pose.pose.orientation.x = q[0];
  marker_pose.pose.orientation.y = q[1];
  marker_pose.pose.orientation.z = q[2];


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

   geometry_msgs::PoseStamped marker_pose_in_quickcoupler_frame;
   //transform pose from chassis/world frame to quickcoupler frame
   listener.waitForTransform("quickcoupler","chassis", ros::Time::now(), ros::Duration(0.1)); //avoid time difference exception
   listener.transformPose("quickcoupler",ros::Time(0), marker_pose_in_world_frame, "chassis", marker_pose_in_quickcoupler_frame);

   marker_pose_in_quickcoupler_frame.pose.position.x -= 0.25 + quickcoupler_x;

   //transform pose from quickcoupler frame to chassis/world frame
   listener.waitForTransform("chassis","quickcoupler", ros::Time::now(), ros::Duration(0.1)); //avoid time difference exception
   listener.transformPose("chassis",ros::Time(0), marker_pose_in_quickcoupler_frame, "quickcoupler", marker_pose_in_world_frame);

   akitGroup.setPoseTarget(marker_pose_in_world_frame.pose);
   success = (akitGroup.plan(motionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   if(success)
     akitGroup.execute(motionPlan);

   moveit::core::RobotStatePtr current_state = akitGroup.getCurrentState();
   std::vector<double> joint_group_positions;
   current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

   const std::vector<std::string> &jointNames = joint_model_group->getVariableNames();

   joint_group_positions[4] += M_PI/2; //rotate gripper
   akitGroup.setJointValueTarget(joint_group_positions);
   success = (akitGroup.plan(motionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   if(success)
     akitGroup.execute(motionPlan);

   for (std::size_t i = 0; i < jointNames.size(); ++i){
     ROS_INFO("joint %s: %f", jointNames[i].c_str(), joint_group_positions[i]);
    }
}




