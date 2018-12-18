#include <akit_pick_place/akit_pick_place.h>


geometry_msgs::Pose akit_pick_place::interactive_pose;
std::string akit_pick_place::interactive_name;

akit_pick_place::akit_pick_place(){

  //check all parameters are loaded

  if (!nh.hasParam("/planning_group")){
    ROS_ERROR("planning_group parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/planning_group", PLANNING_GROUP);

  if (!nh.hasParam("/world_frame")){
    ROS_ERROR("world_frame parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/world_frame", WORLD_FRAME);

  if (!nh.hasParam("/eef_group")){
    ROS_ERROR("eef_group parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/eef_group", EEF_GROUP);

  if (!nh.hasParam("/base_link")){
    ROS_ERROR("base_link parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/base_link", BASE_LINK);

  if (!nh.hasParam("/eef_parent_link")){
    ROS_ERROR("eef_parent_link parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/eef_parent_link", EEF_PARENT_LINK);

  if (!nh.hasParam("/gripper_frame")){
    ROS_ERROR("gripper_frame parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/gripper_frame", GRIPPER_FRAME);

  if (!nh.hasParam("/bucket_frame")){
    ROS_ERROR("bucket_frame parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/bucket_frame", BUCKET_FRAME);

  if (!nh.hasParam("/gripper_length")){
    ROS_ERROR("gripper_length parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/gripper_length", GRIPPER_LENGTH);

  if (!nh.hasParam("/gripper_jaw_length")){
    ROS_ERROR("gripper_jaw_length parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/gripper_jaw_length", GRIPPER_JAW_LENGTH);

  if (!nh.hasParam("/gripper_side_length")){
    ROS_ERROR("gripper_side_length parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/gripper_side_length", GRIPPER_SIDE_LENGTH);

  waypoints = std::vector<geometry_msgs::Pose>(1);

  akitGroup = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

  akitJointModelGroup = akitGroup->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  gripperGroup = new moveit::planning_interface::MoveGroupInterface(EEF_GROUP);

  gripperJointModelGroup = gripperGroup->getCurrentState()->getJointModelGroup(EEF_GROUP);

  akitState = akitGroup->getCurrentState();

  gripperState = gripperGroup->getCurrentState();

  gripperState->copyJointGroupPositions(gripperJointModelGroup,gripperJointPositions);

  akitState->copyJointGroupPositions(akitJointModelGroup,akitJointPositions);

  robotModelLoader.reset(new robot_model_loader::RobotModelLoader("robot_description"));

  robotModelPtr = robotModelLoader->getModel();

  planningScenePtr.reset(new planning_scene::PlanningScene(robotModelPtr));

  server.reset(new interactive_markers::InteractiveMarkerServer("akit_pick_place","",false));

  visual_tools.reset(new moveit_visual_tools::MoveItVisualTools(BASE_LINK, "visualization_marker"));

  planning_scene_diff_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");

  planning_scene_diff_client_ = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");

  get_planning_scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",10);

  pose_pub = nh.advertise<geometry_msgs::PoseArray>("publish_pose", 10);
}

//destructor
akit_pick_place::~akit_pick_place(){
}

void akit_pick_place::writeOutputPlanningTime(std::string file_name){
  ROS_INFO_STREAM("Planning time = " << MotionPlan.planning_time_);

  std::ofstream fout;
  fout.open(file_name.c_str(), std::ofstream::out | std::ofstream::app);
  fout << MotionPlan.planning_time_ << std::endl;
  fout.close();
}

void akit_pick_place::writeOutputTrajectoryLength(std::string file_name){

  std::ofstream fout;
  fout.open(file_name.c_str(), std::ofstream::out | std::ofstream::app);

  robot_trajectory::RobotTrajectory traj_obj(akitGroup->getRobotModel(), akitGroup->getName());
  moveit::core::RobotState ref_state_obj(akitGroup->getRobotModel());
  traj_obj.setRobotTrajectoryMsg(ref_state_obj, MotionPlan.trajectory_);

  const std::string tool_link = akitGroup->getLinkNames().back();
  auto euclidean_distance = [&tool_link](const robot_state::RobotState& state1,const robot_state::RobotState& state2){
  auto tool_pose1 = state1.getGlobalLinkTransform(tool_link);
  auto tool_pose2 = state2.getGlobalLinkTransform(tool_link);
  return (tool_pose1.translation() - tool_pose2.translation()).norm();
  };

  double traj_length_cartesian_space = 0.0000;
  for (size_t p = 1; p < traj_obj.getWayPointCount(); ++p)
  {
    traj_length_cartesian_space += euclidean_distance(traj_obj.getWayPoint(p), traj_obj.getWayPoint(p-1));
  }
  fout << traj_length_cartesian_space << std::endl;
  ROS_INFO_STREAM("trajectory_length = " << traj_length_cartesian_space);
  fout.close();
}

void akit_pick_place::displayTrajectory(moveit::planning_interface::MoveGroupInterface::Plan motion_plan,
                                        geometry_msgs::Pose axis_pose, std::string axis_name,
                                        rviz_visual_tools::colors color){

  visual_tools->publishAxisLabeled(axis_pose, axis_name , rviz_visual_tools::scales::LARGE);
  visual_tools->publishTrajectoryLine(motion_plan.trajectory_, akitJointModelGroup,color);
  visual_tools->trigger();
}

void akit_pick_place::generateGrasps(geometry_msgs::Pose block_pose_, double block_size_,bool sideGrasps, bool visualize){

  geometry_msgs::PoseStamped box_in_chassis_frame,box_in_world_frame;

  box_in_world_frame.pose = block_pose_;
  box_in_world_frame.header.frame_id = WORLD_FRAME;

  //transform object pose from world frame to chassis frame
  transform_listener.waitForTransform(BASE_LINK,WORLD_FRAME, ros::Time::now(), ros::Duration(0.1)); //avoid time difference exception
  transform_listener.transformPose(BASE_LINK,ros::Time(0), box_in_world_frame, WORLD_FRAME, box_in_chassis_frame);

  //create yaw angle (rotation around z-axis)
  double yaw = atan2(box_in_chassis_frame.pose.position.y,box_in_chassis_frame.pose.position.x);

  if (!sideGrasps){

    side_grasps = false;
    //calculate length between base frame origin to object frame
    double line_length = sqrt(pow(box_in_chassis_frame.pose.position.x,2)+pow(box_in_chassis_frame.pose.position.y,2));
    double number_of_steps = 10.0;

    //grasp distance covered = length of block hypotenuse + 2*gripper side length
    double blockHypotenuse = sqrt(pow(block_size_,2)+pow(block_size_,2));
    double covered_distance = blockHypotenuse + (2 * GRIPPER_SIDE_LENGTH);
    double starting_point = line_length - (blockHypotenuse/2) - GRIPPER_SIDE_LENGTH;
    double step_size = covered_distance / number_of_steps;

    tf::Quaternion q = tf::createQuaternionFromRPY(0.0,0.0,yaw); //fix rotation to be only around z-axis
    for (double i = step_size; i <= covered_distance; i += step_size){
      grasp_pose.position.x = (starting_point + i) * cos(yaw);
      grasp_pose.position.y = (starting_point + i) * sin(yaw);
      grasp_pose.position.z = GRIPPER_LENGTH + box_in_chassis_frame.pose.position.z + (block_size_/2); //divide by 2 --> center of gravity of cube
      grasp_pose.orientation.x = q[0];
      grasp_pose.orientation.y = q[1];
      grasp_pose.orientation.z = q[2];
      grasp_pose.orientation.w = q[3];
      grasp_pose_vector.push_back(grasp_pose);
    }

    //visualization of grasp points
    if(visualize){
      this->visualizeGrasps(grasp_pose_vector, BASE_LINK);
    }

  } else { //side grasps

    side_grasps = true;
    double blockHypotenuse = sqrt(pow(block_size_,2)+pow(block_size_,2)+pow(block_size_,2) ); //internal diagonal of cube
    double pitch_min =  - M_PI / 3; //60 deg
    double pitch_max = - M_PI / 9;  //20 deg
    double angle_incr= M_PI / 90;   // step --> 2 deg --> 20 steps

    double pos_pitch = M_PI / 9;    //20 deg
    double roll = 0.0;

    //circle in xz plane tilted around z-axis ,0.05 to avoid collision
    double radius = GRIPPER_LENGTH + (blockHypotenuse/2) + 0.05;

    //start grasp generation
    for (double pitch = pitch_min; pitch <= pitch_max; pitch += angle_incr, pos_pitch += angle_incr){

      tf::Quaternion q = tf::createQuaternionFromRPY(roll,pitch,yaw);
      grasp_pose.orientation.x = q[0];
      grasp_pose.orientation.y = q[1];
      grasp_pose.orientation.z = q[2];
      grasp_pose.orientation.w = q[3];
      //semi-circle in xz-plane with a tilt around z-axis (rotation matrix)
      grasp_pose.position.x = box_in_chassis_frame.pose.position.x - radius * cos(pos_pitch) * cos(yaw);
      grasp_pose.position.y = box_in_chassis_frame.pose.position.y -  radius * cos(pos_pitch) * sin(yaw);
      grasp_pose.position.z = box_in_chassis_frame.pose.position.z  + radius * sin(pos_pitch);
      grasp_pose_vector.push_back(grasp_pose);
    }
    if(visualize){
      this->visualizeGrasps(grasp_pose_vector, BASE_LINK);
    }
  }
}

void akit_pick_place::generateGrasps(geometry_msgs::Pose cuboid_pose_, double cuboid_x_, double cuboid_y_, double cuboid_z_, bool sideGrasps, bool visualize){

  //calculate roll,pitch,yaw of the object relative to the world frame for test variable (top grasping)
  tf::Quaternion qq(cuboid_pose_.orientation.x, cuboid_pose_.orientation.y,cuboid_pose_.orientation.z, cuboid_pose_.orientation.w);
  tf::Matrix3x3 m(qq);
  double roll_, pitch_, yaw_;
  m.getRPY(roll_, pitch_, yaw_);

  //testing if the orientation of the object is greater or lower than 45deg
  double test = sin(M_PI/2 - roll_) * sin(M_PI/2 - pitch_);

  //transformation to base_link (chassis frame)
  geometry_msgs::PoseStamped cuboid_in_chassis_frame,cuboid_in_world_frame;

  cuboid_in_world_frame.pose = cuboid_pose_;
  cuboid_in_world_frame.header.frame_id = WORLD_FRAME;

  //transform object pose from world frame to chassis frame
  transform_listener.waitForTransform(BASE_LINK,WORLD_FRAME, ros::Time::now(), ros::Duration(0.1)); //avoid time difference exception
  transform_listener.transformPose(BASE_LINK,ros::Time(0), cuboid_in_world_frame, WORLD_FRAME, cuboid_in_chassis_frame);

  //create yaw angle (rotation around z-axis) of Grasp points
  double yaw = atan2(cuboid_in_chassis_frame.pose.position.y,cuboid_in_chassis_frame.pose.position.x);
  double cuboidDiagonal = sqrt(pow(cuboid_x_,2)+pow(cuboid_z_,2));
  double cuboidHypotenuse = sqrt(pow(cuboid_x_,2)+pow(cuboid_y_,2));

  if(!sideGrasps){

    side_grasps = false;
    //calculate length between base frame origin to object frame
    double line_length = sqrt(pow(cuboid_in_chassis_frame.pose.position.x,2)+pow(cuboid_in_chassis_frame.pose.position.y,2));
    double number_of_steps = 10.0;

    //grasp distance covered = length of cuboid hypotenuse + 2*gripper_side_length
    double covered_distance = cuboidHypotenuse + (2 * GRIPPER_SIDE_LENGTH);
    double starting_point = line_length - (cuboidHypotenuse/2) - GRIPPER_SIDE_LENGTH;
    double step_size = covered_distance / number_of_steps;

    //grasp_pose_vector = std::vector<geometry_msgs::Pose>(number_of_steps); //initialize
    tf::Quaternion q = tf::createQuaternionFromRPY(0.0,0.0,yaw); //rotation to be only around z-axis
    for (double i = step_size; i <= covered_distance; i += step_size){

      /*if the object's orientation in roll or pitch is between -45deg and 45deg
       *then the added distance is the cuboidDiagonal/2
       *if the orientation is outside this range then added distance is the cuboidHypotenuse/2 */

      if(std::abs(test) >= sin(M_PI/4)){
        grasp_pose.position.z = GRIPPER_LENGTH + cuboid_in_chassis_frame.pose.position.z + (cuboidDiagonal/2);
      } else if (std::abs(test) <= sin(M_PI/4)){
        grasp_pose.position.z = GRIPPER_LENGTH + cuboid_in_chassis_frame.pose.position.z + (cuboidHypotenuse/2);
      }
      grasp_pose.position.x = (starting_point + i) * cos(yaw);
      grasp_pose.position.y = (starting_point + i) * sin(yaw);
      grasp_pose.orientation.x = q[0];
      grasp_pose.orientation.y = q[1];
      grasp_pose.orientation.z = q[2];
      grasp_pose.orientation.w = q[3];
      grasp_pose_vector.push_back(grasp_pose);

    }

    if(visualize){
      this->visualizeGrasps(grasp_pose_vector, BASE_LINK);
    }

 } else { //side grasps

    side_grasps = true;
    double pitch_min = - M_PI / 3;  //60 deg
    double pitch_max = - M_PI / 9;  //20 deg
    double angle_incr= M_PI / 90;   // step --> 2 deg --> 20 steps

    double pos_pitch = M_PI / 9;    //20 deg
    double roll = 0.0; //fix rotation around x-axis to zero

    //make a circle in xz plane tilted around z-axis, 0.05 to avoid collision
    double radius = 0.0;
    if(std::abs(test) >= sin(M_PI/4)){
      radius = GRIPPER_LENGTH + (cuboidDiagonal/2) + 0.05;
    } else if (std::abs(test) <= sin(M_PI/4)){
      radius = GRIPPER_LENGTH + (cuboidHypotenuse/2) + 0.05;
    }
     //start grasp generation
    for (double pitch = pitch_min; pitch <= pitch_max; pitch += angle_incr, pos_pitch += angle_incr){

      tf::Quaternion q = tf::createQuaternionFromRPY(roll,pitch,yaw);
      grasp_pose.orientation.x = q[0];
      grasp_pose.orientation.y = q[1];
      grasp_pose.orientation.z = q[2];
      grasp_pose.orientation.w = q[3];
      //semi-circle in xz-plane with a tilt around z-axis (rotation matrix)
      grasp_pose.position.x = cuboid_in_chassis_frame.pose.position.x - radius * cos(pos_pitch) * cos(yaw);
      grasp_pose.position.y = cuboid_in_chassis_frame.pose.position.y -  radius * cos(pos_pitch) * sin(yaw);
      grasp_pose.position.z = cuboid_in_chassis_frame.pose.position.z + radius * sin(pos_pitch);
      grasp_pose_vector.push_back(grasp_pose);
    }
    if(visualize){
      this->visualizeGrasps(grasp_pose_vector, BASE_LINK);
    }
  }
}

void akit_pick_place::generateGrasps(geometry_msgs::Pose cylinder_pose_, double cylinder_height_, double cylinder_radius_,bool sideGrasps, bool visualize){

  //calculate roll,pitch,yaw of the object relative to the world frame for test variable (top grasping)
  tf::Quaternion qq(cylinder_pose_.orientation.x, cylinder_pose_.orientation.y,cylinder_pose_.orientation.z, cylinder_pose_.orientation.w);
  tf::Matrix3x3 m(qq);
  double roll_, pitch_, yaw_;
  m.getRPY(roll_, pitch_, yaw_);

  ROS_INFO_STREAM("roll = " << roll_ << " pitch = " << pitch_ << "yaw = " << yaw_);

  //testing if the orientation of the object (in x,y) is greater or lower than 45deg
  double test = sin(M_PI/2 - roll_) * sin(M_PI/2 - pitch_);

  //transformation to base_link (chassis frame)
  geometry_msgs::PoseStamped cylinder_in_chassis_frame,cylinder_in_world_frame;

  cylinder_in_world_frame.pose = cylinder_pose_;
  cylinder_in_world_frame.header.frame_id = WORLD_FRAME;

  //transform object pose from world frame to chassis frame
  transform_listener.waitForTransform(BASE_LINK,WORLD_FRAME, ros::Time::now(), ros::Duration(0.1)); //avoid time difference exception
  transform_listener.transformPose(BASE_LINK,ros::Time(0), cylinder_in_world_frame, WORLD_FRAME, cylinder_in_chassis_frame);

  //create yaw angle (rotation around z-axis) of grasp points
  double yaw = atan2(cylinder_in_chassis_frame.pose.position.y,cylinder_in_chassis_frame.pose.position.x);
  double cylinderInternalDiagonal = sqrt(pow(2 * cylinder_radius_,2)+pow(cylinder_height_,2));

  if (!sideGrasps){

    side_grasps = false;
    //calculate length between base frame origin to object frame
    double line_length = sqrt(pow(cylinder_in_chassis_frame.pose.position.x,2)+pow(cylinder_in_chassis_frame.pose.position.y,2));
    double number_of_steps = 10.0;

    //grasp distance covered = diameter + 2*gripper side length
    double cylinderDiameter = 2 * cylinder_radius_;
    double covered_distance = cylinderDiameter + (2 * GRIPPER_SIDE_LENGTH);
    double starting_point = line_length - cylinder_radius_ - GRIPPER_SIDE_LENGTH;
    double step_size = covered_distance / number_of_steps;

    tf::Quaternion q = tf::createQuaternionFromRPY(0.0,0.0,yaw); //fix rotation to be only around z-axis
    for (double i = step_size; i <= covered_distance; i += step_size){

      /*if the object's orientation in world frame --> roll or pitch is between -45deg and 45deg
       *then the added distance is the cylinderInternalDiagonal/2
       *if the orientation is outside this range then added distance is the cylinder radius*/

      if(std::abs(test) >= sin(M_PI/4)){
        grasp_pose.position.z = GRIPPER_LENGTH + cylinder_in_chassis_frame.pose.position.z + (cylinderInternalDiagonal/2);
      } else if (std::abs(test) <= sin(M_PI/4)){
        grasp_pose.position.z = GRIPPER_LENGTH + cylinder_in_chassis_frame.pose.position.z + (cylinder_radius_);
      }
      grasp_pose.position.x = (starting_point + i) * cos(yaw);
      grasp_pose.position.y = (starting_point + i) * sin(yaw);
      grasp_pose.orientation.x = q[0];
      grasp_pose.orientation.y = q[1];
      grasp_pose.orientation.z = q[2];
      grasp_pose.orientation.w = q[3];
      grasp_pose_vector.push_back(grasp_pose);

    }

    if(visualize){
      this->visualizeGrasps(grasp_pose_vector, BASE_LINK);
    }

  } else { //side grasps

    side_grasps = true;
    double pitch_min = - M_PI / 3;  //60 deg
    double pitch_max = - M_PI / 9;  //20 deg
    double angle_incr= M_PI / 90;   // step --> 2 deg --> 20 steps

    double pos_pitch = M_PI / 9;    //20 deg
    double roll = 0.0; //fix rotation around x-axis to zero

    //make a circle in xz plane tilted around z-axis, tolerance 0.05 to avoid collision
    double radius = 0.0;
    if(std::abs(test) >= sin(M_PI/4)){
      radius = GRIPPER_LENGTH + (cylinderInternalDiagonal/2) + 0.05;
    } else if (std::abs(test) <= sin(M_PI/4)){
      radius = GRIPPER_LENGTH + cylinder_radius_ + 0.05;
    }
    //start grasp generation
    for (double pitch = pitch_min; pitch <= pitch_max; pitch += angle_incr, pos_pitch += angle_incr){

      tf::Quaternion q = tf::createQuaternionFromRPY(roll,pitch,yaw);
      grasp_pose.orientation.x = q[0];
      grasp_pose.orientation.y = q[1];
      grasp_pose.orientation.z = q[2];
      grasp_pose.orientation.w = q[3];
      //semi-circle in xz-plane with a tilt around z-axis (rotation matrix)
      grasp_pose.position.x = cylinder_in_chassis_frame.pose.position.x - radius * cos(pos_pitch) * cos(yaw);
      grasp_pose.position.y = cylinder_in_chassis_frame.pose.position.y -  radius * cos(pos_pitch) * sin(yaw);
      grasp_pose.position.z = cylinder_in_chassis_frame.pose.position.z + radius * sin(pos_pitch);
      grasp_pose_vector.push_back(grasp_pose);
    }
    if(visualize){
      this->visualizeGrasps(grasp_pose_vector, BASE_LINK);
    }
  }
}

void akit_pick_place::visualizeGrasps(std::vector<geometry_msgs::Pose> points, std::string frame){

  ROS_INFO_STREAM("---------- Points Visualization ----------");
  uint32_t shape = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = frame;
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
  for (int i = 0; i < points.size(); ++i){
    marker.id = i;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = points[i];
    while (marker_pub.getNumSubscribers() < 1)
    {
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
  }
}
bool akit_pick_place::rotateGripper(double angle_rad){

  //update start state to current state
  gripperState = gripperGroup->getCurrentState();
  gripperGroup->setStartState(*gripperState);

  gripperJointPositions[0] += angle_rad; //gripper rotator joint
  gripperGroup->setJointValueTarget(gripperJointPositions);

  //plan and execute motion plan
  gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (gripperSuccess){
    gripperGroup->execute(gripperMotionPlan);
    ROS_INFO_STREAM("Gripper Motion Plan: " << (gripperSuccess ? "Rotated gripper" : "FAILED TO ROTATE GRIPPER"));
  }

  return (gripperSuccess ? true : false);
}

bool akit_pick_place::rotateGripper(std::string object_id){ //needs adjusting !!

  //get collision object from planning scene
  std::vector<std::string> object_id_(1,object_id);
  std::map<std::string, moveit_msgs::CollisionObject> object = planningSceneInterface.getObjects(object_id_);
  std::map<std::string, moveit_msgs::CollisionObject>::iterator object_it = object.find(object_id);

  //update start state to current state
  gripperState = gripperGroup->getCurrentState();
  gripperGroup->setStartState(*gripperState);

  geometry_msgs::PoseStamped object_in_world_frame, object_in_gripper_frame;
  object_in_world_frame.pose = object_it->second.primitive_poses[0];
  object_in_world_frame.header.frame_id = object_it->second.header.frame_id;

  //transform object from world frame to gripper rotator frame, wait to avoid time difference exceptions
  transform_listener.waitForTransform(GRIPPER_FRAME, WORLD_FRAME, ros::Time::now(), ros::Duration(0.1));
  transform_listener.transformPose(GRIPPER_FRAME,ros::Time(0), object_in_world_frame, WORLD_FRAME, object_in_gripper_frame);

  //get roll, pitch, yaw between object frame and gripper frame
  tf::Quaternion qq(object_in_gripper_frame.pose.orientation.x, object_in_gripper_frame.pose.orientation.y,
                    object_in_gripper_frame.pose.orientation.z, object_in_gripper_frame.pose.orientation.w);

  //get rotation matrix from quaternion
  tf::Matrix3x3 m(qq);

  //get roll,pitch,yaw from rotation matrix
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  //account for angles in different quadrants
  if (yaw <= 0.0){
    gripperJointPositions[0] = (M_PI/2) + yaw;
  } else {
    gripperJointPositions[0] =  yaw - (M_PI/2);
  }

  gripperGroup->setJointValueTarget(gripperJointPositions);

  //plan and execute motion plan
  gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (gripperSuccess){
    gripperGroup->execute(gripperMotionPlan);
    ROS_INFO_STREAM("Gripper Motion Plan: " << (gripperSuccess ? "Rotated gripper" : "FAILED TO ROTATE GRIPPER"));
  }

  return (gripperSuccess ? true : false);
}

bool akit_pick_place::openGripper(){

  //update start state to current state
  gripperState = gripperGroup->getCurrentState();
  gripperGroup->setStartState(*gripperState);

  //clear joint position vector
  gripperJointPositions.clear();

  //get parameters from server
  std::vector<double> open_gripper;

  if (!nh.hasParam("/open_gripper")){
    ROS_ERROR("open gripper parameter not loaded, did you load grasping parameters .yaml file ?");
    return false;
    exit(1);
  }

  nh.getParam("/open_gripper", open_gripper);

  for (std::size_t i = 0; i < open_gripper.size(); ++i){
    gripperJointPositions.push_back(open_gripper[i]);
  }

  gripperGroup->setJointValueTarget(gripperJointPositions);

  //plan and execute motion plan
  gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (gripperSuccess){
    gripperGroup->execute(gripperMotionPlan);
    ROS_INFO_STREAM("Gripper Motion Plan: " << (gripperSuccess ? "Opened gripper" : "FAILED TO OPEN GRIPPER"));
  }
  return (gripperSuccess ? true : false);
}

bool akit_pick_place::closeGripper(std::string object_id){

  //update start state to current state
  gripperState = gripperGroup->getCurrentState();
  gripperGroup->setStartState(*gripperState);

  //relate close gripper to the side lengths of the object --> gripper close angle is related to the minimum side

  //parameters
  double max_open_length, max_open_gripper_joint_value, tolerance;

  if (!nh.hasParam("/max_open_length")){
    ROS_ERROR("max_open_length parameter (distance between gripper jaws when fully opened) "
              "not loaded, did you load grasping parameters .yaml file ?");
    return false;
    exit(1);
  }

  nh.getParam("/max_open_length", max_open_length);

  if (!nh.hasParam("/max_open_gripper_joint_value")){
    ROS_ERROR("max_open_gripper_joint_value parameter (maximum value for gripper joint when opened) "
              "not loaded, did you load grasping parameters .yaml file ?");
    return false;
    exit(1);
  }

  nh.getParam("/max_open_gripper_joint_value", max_open_gripper_joint_value);

  if (!nh.hasParam("/close_gripper_tolerance")){
    ROS_WARN("close_gripper_tolerance parameter (maximum value for gripper joint when opened) "
              "not loaded, did you load grasping parameters .yaml file ? --> using tolerance = 0.0");
    tolerance = 0.0;

  } else {

    nh.getParam("/close_gripper_tolerance", tolerance);

  }

  //get collision object from planning scene
  std::vector<std::string> object_id_(1,object_id);
  std::map<std::string, moveit_msgs::CollisionObject> object = planningSceneInterface.getObjects(object_id_);
  std::map<std::string, moveit_msgs::CollisionObject>::iterator object_it = object.find(object_id);


  //compute minimum side
  double min_side = object_it->second.primitives[0].dimensions[0];

  for(std::size_t i = 0; i < object_it->second.primitives[0].dimensions.size(); ++i){
    if (object_it->second.primitives[0].dimensions[i] < min_side){
      min_side = object_it->second.primitives[0].dimensions[i];
    }
  }
   //to get diameter not radius --> not needed for cubes and cuboids --> will need further adjusting with vision
  if(object_it->second.primitives[0].type == shape_msgs::SolidPrimitive::CYLINDER){
    min_side *= 2;
  }

  double gripper_close_angle = ((min_side * max_open_gripper_joint_value) / max_open_length) - tolerance;

  //get joints from parameter server
  std::vector<std::string> gripper_joint_names =  gripperGroup->getJointNames();
  std::vector<std::string> close_gripper_joints;
  std::vector<std::string>::iterator it1;
  std::vector<std::string>::difference_type it2;

  if (!nh.hasParam("/close_gripper_joints")){
    ROS_ERROR("close_gripper_joints parameter not loaded, did you load grasping parameters .yaml file ?");
    return false;
    exit(1);
  }

  nh.getParam("/close_gripper_joints", close_gripper_joints);

  //find index in vector matching parameters in server
  for (std::size_t i = 0; i < close_gripper_joints.size(); ++i){
    it1  = std::find(gripper_joint_names.begin(), gripper_joint_names.end(), close_gripper_joints[i]);
    it2  = std::distance(gripper_joint_names.begin(), it1);
    gripperJointPositions[it2] = gripper_close_angle;
  }

  gripperGroup->setJointValueTarget(gripperJointPositions);

  //plan and execute motion plan
  gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (gripperSuccess){
    gripperGroup->execute(gripperMotionPlan);
    ROS_INFO_STREAM("Gripper Motion Plan: " << (gripperSuccess ? "Closed gripper" : "FAILED TO CLOSE GRIPPER"));
  }
  return (gripperSuccess ? true : false);
}

bool akit_pick_place::executeAxisCartesianMotion(bool direction, double cartesian_distance, char axis){

  //update start state to current state
  akitState = akitGroup->getCurrentState();
  akitGroup->setStartState(*akitState);

  //UP = true, DOWN = false
  geometry_msgs::PoseStamped pose_in_base_frame, pose_in_quickcoupler_frame;

  pose_in_base_frame.pose = akitGroup->getCurrentPose(EEF_PARENT_LINK).pose; //chassis frame
  pose_in_base_frame.header.frame_id = BASE_LINK; //pose stamped

  //transform from base link frame to quickcoupler frame, wait to avoid time difference exceptions
  transform_listener.waitForTransform(EEF_PARENT_LINK, BASE_LINK, ros::Time::now(), ros::Duration(0.1));
  transform_listener.transformPose(EEF_PARENT_LINK,ros::Time(0), pose_in_base_frame, BASE_LINK, pose_in_quickcoupler_frame);

  if (!direction){        //downwards cartesian motion in quickcoupler frame
    switch (axis){
    case 'x':
      pose_in_quickcoupler_frame.pose.position.x -= cartesian_distance;
      break;
    case 'y':
      pose_in_quickcoupler_frame.pose.position.y -= cartesian_distance;
      break;
    case 'z':
      pose_in_quickcoupler_frame.pose.position.z -= cartesian_distance;
      break;
    }
  } else {                //upwards cartesian motion in quickcoupler frame
    switch (axis){
    case 'x':
      pose_in_quickcoupler_frame.pose.position.x += cartesian_distance;
      break;
    case 'y':
      pose_in_quickcoupler_frame.pose.position.y += cartesian_distance;
      break;
    case 'z':
      pose_in_quickcoupler_frame.pose.position.z += cartesian_distance;
      break;
    }
  }
  //transform back to base frame, wait to avoid time difference exceptions
  transform_listener.waitForTransform(BASE_LINK, EEF_PARENT_LINK, ros::Time::now(), ros::Duration(0.1));
  transform_listener.transformPose(BASE_LINK,ros::Time(0), pose_in_quickcoupler_frame, EEF_PARENT_LINK, pose_in_base_frame);

  waypoints[0] = pose_in_base_frame.pose;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  akitGroup->setMaxVelocityScalingFactor(0.1);

  double fraction  = akitGroup->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_STREAM("Visualizing Cartesian Motion plan:  " << (fraction * 100.0) <<"%% achieved");

  if (fraction * 100 >= 50.0){
    MotionPlan.trajectory_ = trajectory;
    ROS_INFO_STREAM("---------- Executing Cartesian Motion ----------");
    akitSuccess = (akitGroup->execute(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("Cartesian Motion Plan: " << (akitSuccess ? "Executed" : "FAILED"));
    return (akitSuccess ? true : false);
  } else {
    ROS_ERROR("Cannot execute cartesian motion, plan < 50 %%");
    return false;
  }
}

//executes first pose reached in input position vector --> modify to execute highest score in vector
bool akit_pick_place::planAndExecuteCartesianGoals(std::vector<geometry_msgs::Pose> poses, std::string pose){

  //update start state to current state
  akitState = akitGroup->getCurrentState();
  akitGroup->setStartState(*akitState);

  int count = 0;
  bool executed, found_ik;

  for(std::size_t i = 0; i < poses.size(); ++i){

    //convert positions from cartesian space to joint space to work with all planners
    found_ik =  akitState->setFromIK(akitJointModelGroup, poses[i], 10,0.0);
    if (found_ik)
    {
      ROS_INFO_STREAM("Found IK for " << pose << " position " << count << " successfully");
      akitState->copyJointGroupPositions(akitJointModelGroup, akitJointPositions);
      akitGroup->setJointValueTarget(akitJointPositions);
    } else
    {
      ROS_ERROR_STREAM("Failed to find inverse kinematics for " << pose << " position " << count);
      count++;
      if (count == poses.size()){
        ROS_ERROR_STREAM("Failed to plan to " << pose << " position");
        return false;
        exit(1);
      }
      continue;
    }
    akitSuccess = (akitGroup->plan(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("Motion Plan: " << (akitSuccess ? "Successful" : "FAILED"));
    if(!akitSuccess){
        ROS_ERROR_STREAM("Failed to plan to " << pose << " position");
        return false;
        exit(1);
    } else {
      this->displayTrajectory(MotionPlan,poses[i], pose + std::to_string(count), rviz_visual_tools::colors::ORANGE);
      executed = (akitGroup->execute(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_STREAM("Executing Motion plan to " << pose << " position, "<< (executed ? "Executed" : "FAILED"));
      if (!executed){
        ROS_ERROR_STREAM("Failed to execute motion plan to " << pose << " position");
        return false;
        exit(1);
      } else {
        return true;
        break;
      }
    }
  }
}

bool akit_pick_place::planAndExecuteJointGoals(std::string group, std::vector<double> joint_states, bool add_to_current_joint_states){

  //update arm start state to current state
  akitState = akitGroup->getCurrentState();
  akitGroup->setStartState(*akitState);

  //update gripper start state to current state
  gripperState = gripperGroup->getCurrentState();
  gripperGroup->setStartState(*gripperState);

  if (group == PLANNING_GROUP) {
    akitState->copyJointGroupPositions(akitJointModelGroup,akitJointPositions);
    if (add_to_current_joint_states){
      for (std::size_t i = 0; i < joint_states.size(); ++i){
        akitJointPositions[i] += joint_states[i];
      }
    } else {
      for (std::size_t i = 0; i < joint_states.size(); ++i){
        akitJointPositions[i] = joint_states[i];
      }
    }

    akitGroup->setJointValueTarget(akitJointPositions);
    akitSuccess = (akitGroup->plan(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (akitSuccess){
        akitGroup->execute(MotionPlan);
        ROS_INFO_STREAM("executed arm joint space motion");
      } else {
        ROS_ERROR_STREAM("failed to execute arm joint space motion");
        return false;
        exit(1);
      }
   } else if (group == EEF_GROUP) { //for robots where the rotator joint is in the gripper group not in the arm, needs further testing whether to remove or leave

    gripperState->copyJointGroupPositions(gripperJointModelGroup,gripperJointPositions);
    if (add_to_current_joint_states){
      for (std::size_t i = 0; i < joint_states.size(); ++i){
        gripperJointPositions[i] += joint_states[i];
      }
    } else {
      for (std::size_t i = 0; i < joint_states.size(); ++i){
        gripperJointPositions[i] = joint_states[i];
      }
    }

    gripperGroup->setJointValueTarget(gripperJointPositions);
    gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (gripperSuccess){
        gripperGroup->execute(gripperMotionPlan);
        ROS_INFO_STREAM("executed gripper joint space motion");
      } else {
        ROS_ERROR_STREAM("failed to execute gripper joint space motion");
        return false;
        exit(1);
      }
   }
}


//allow gripper to touch object to be picked
bool akit_pick_place::allowObjectCollision(std::string object_id){
  acm = planningScenePtr->getAllowedCollisionMatrix();

  std::vector<std::string> gripper_links = gripperGroup->getLinkNames();
  for(std::size_t i = 0;i < gripper_links.size(); ++i)
  {
    acm.setEntry(gripper_links[i], object_id, true);
  }
  acm.getMessage(planning_scene_msg_.allowed_collision_matrix);
  planning_scene_msg_.is_diff = true;
  planning_scene_srv.request.scene = planning_scene_msg_;

  if (planning_scene_diff_client.call(planning_scene_srv)){
      return true;
    } else {
      return false;
  }
}

//reset acm to restore object as a collision object
bool akit_pick_place::resetAllowedCollisionMatrix(std::string object_id){
  acm = planningScenePtr->getAllowedCollisionMatrix();

  std::vector<std::string> gripper_links = gripperGroup->getLinkNames();
  for(std::size_t i = 0;i < gripper_links.size(); ++i){
    acm.removeEntry(gripper_links[i], object_id);
  }

  acm.getMessage(planning_scene_msg_.allowed_collision_matrix);
  planning_scene_msg_.is_diff = true;
  planning_scene_srv.request.scene = planning_scene_msg_;
  if (planning_scene_diff_client.call(planning_scene_srv)){
      return true;
    } else {
      return false;
  }
}

//allow collision during tool exchange
bool akit_pick_place::allowToolCollision(std::string tool_id){
  acm = acm = planningScenePtr->getAllowedCollisionMatrix();
  std::transform(tool_id.begin(), tool_id.end(), tool_id.begin(), ::tolower);

  if (tool_id != GRIPPER_FRAME && tool_id != BUCKET_FRAME){
    ROS_ERROR_STREAM("Unknown tool, please write correct tool name");
    return false;
    exit(1);
  }

  std::vector<std::string> group_links;

  if (!nh.hasParam(PLANNING_GROUP)){
    ROS_ERROR_STREAM(PLANNING_GROUP + " parameter not loaded, did you load planning groups yaml file ?");
    return false;
    exit(1);
  }

  nh.getParam(PLANNING_GROUP, group_links);

  for (std::size_t i= 0; i < group_links.size(); ++i){
    acm.setEntry(group_links[i], tool_id, true);
}
  acm.getMessage(planning_scene_msg_.allowed_collision_matrix);
  planning_scene_msg_.is_diff = true;
  planning_scene_srv.request.scene = planning_scene_msg_;

  if (planning_scene_diff_client.call(planning_scene_srv)){
      return true;
    } else {
      return false;
  }
}

bool akit_pick_place::attachCollisionObject(std::string object_id){

  //get collision object from planning scene
  std::vector<std::string> object_id_(1,object_id);
  std::map<std::string, moveit_msgs::CollisionObject> object = planningSceneInterface.getObjects(object_id_);
  std::map<std::string, moveit_msgs::CollisionObject>::iterator object_it = object.find(object_id);

  //create attached collision object + planning scene instances
  moveit_msgs::AttachedCollisionObject attached_collision_object;
  moveit_msgs::ApplyPlanningScene planningSceneSrv_;
  moveit_msgs::PlanningScene planning_scene;

  //filling attached collision object instance
  attached_collision_object.object.header.frame_id = object_it->second.header.frame_id;
  attached_collision_object.object.id = object_it->second.id;
  attached_collision_object.link_name = GRIPPER_FRAME;
  attached_collision_object.object.primitives.push_back(object_it->second.primitives[0]);
  attached_collision_object.object.primitive_poses.push_back(object_it->second.primitive_poses[0]);
  attached_collision_object.object.operation = object_it->second.operation; //operation bit is ADD

  //remove collision object from enviroment
  ROS_INFO_STREAM("removing collision object from the enviroment");
  attached_collision_object.object.operation = attached_collision_object.object.REMOVE; //operation bit is REMOVE from world

  //add "removed" collisin object to planning scene
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(attached_collision_object.object);

  //add collision object to attached collision objects
  attached_collision_object.object.operation = attached_collision_object.object.ADD; //operation bit is ADD to attached
  planning_scene.robot_state.attached_collision_objects.push_back(attached_collision_object);
  planning_scene.is_diff = true;

  //call service
  planningSceneSrv_.request.scene = planning_scene;
  planning_scene_diff_client_.call(planningSceneSrv_);

  if (planningSceneSrv_.response.success == true){

    ROS_INFO_STREAM("successfully attached collision object to gripper");
    return true;

  } else {

    ROS_INFO_STREAM("failed to attach collision object to gripper");
    return false;
  }
}

//detach object to gripper using planning scene services
bool akit_pick_place::detachCollisionObject(std::string object_id){

  //get collision object from planning scene
  std::vector<std::string> object_id_(1,object_id);
  std::map<std::string, moveit_msgs::CollisionObject> object = planningSceneInterface.getObjects(object_id_);
  std::map<std::string, moveit_msgs::CollisionObject>::iterator object_it = object.find(object_id);

  //remove first from attached objects and add to the planning scene

  //create necessary objects for services
  moveit_msgs::AttachedCollisionObject detached_collision_object;
  moveit_msgs::ApplyPlanningScene planningSceneSrv_;
  moveit_msgs::PlanningScene planning_scene;
  moveit_msgs::GetPlanningScene get_planning_scene_srv; //get planning scene service client object

  //get robot state from service call --> returns  CURRENT attached collision object info so that object is returned to env at correct pose
  get_planning_scene_srv.request.components.components = 4;
  get_planning_scene_client.call(get_planning_scene_srv);

  //filling detached collision object instance
  detached_collision_object.object.header.frame_id = get_planning_scene_srv.response.scene.robot_state.attached_collision_objects[0].object.header.frame_id;
  detached_collision_object.object.id = get_planning_scene_srv.response.scene.robot_state.attached_collision_objects[0].object.id;
  detached_collision_object.link_name = get_planning_scene_srv.response.scene.robot_state.attached_collision_objects[0].link_name;
  detached_collision_object.object.primitives.push_back(get_planning_scene_srv.response.scene.robot_state.attached_collision_objects[0].object.primitives[0]);
  detached_collision_object.object.primitive_poses.push_back(get_planning_scene_srv.response.scene.robot_state.attached_collision_objects[0].object.primitive_poses[0]);
  detached_collision_object.object.operation = object_it->second.operation; //operation bit is ADD

  //removing from attached collision objects
  detached_collision_object.object.operation = detached_collision_object.object.REMOVE; //operation bit is REMOVE from attached
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detached_collision_object);
  planning_scene.robot_state.is_diff = true;

  //adding to world again
  detached_collision_object.object.operation = detached_collision_object.object.ADD; //operation bit is ADD to world again
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(detached_collision_object.object); //spawns the object relative to gripper frame !! check
  planning_scene.is_diff = true;

  planningSceneSrv_.request.scene = planning_scene;
  planning_scene_diff_client_.call(planningSceneSrv_);

  if (planningSceneSrv_.response.success == true){

    ROS_INFO_STREAM("successfully detached collision object from gripper");
    return true;

  } else {

    ROS_INFO_STREAM("failed to detach collision object from gripper");
    return false;
  }
}

bool akit_pick_place::pick(std::string object_id, bool new_grasp_generation){ //temp bool until akit is finished
  ROS_INFO_STREAM("---------- Starting Pick Routine ----------");

  //update start state to current state
  akitState = akitGroup->getCurrentState();
  akitGroup->setStartState(*akitState);

  //move from home position to pre-grasp position
  if (new_grasp_generation){

      //transform grasps to base link frame
      ROS_INFO_STREAM("Transforming grasps to base link frame");
      std::vector<geometry_msgs::PoseStamped> transformed_grasps = this->transformGrasps(grasps);
      std::vector<geometry_msgs::Pose> poses;
      poses.clear();

      for (std::size_t i = 0; i < transformed_grasps.size(); ++i){
          poses.push_back(transformed_grasps[i].pose);
      }

      //loop through all grasp poses
      if(!this->planAndExecuteCartesianGoals(poses, "pre_grasp")){
        ROS_ERROR("Failed to plan and execute");
        return false;
        exit(1);
      }

    } else {

      //loop through all grasp poses
      if(!this->planAndExecuteCartesianGoals(grasp_pose_vector, "pre_grasp")){
        ROS_ERROR("Failed to plan and execute");
        grasp_pose_vector.clear();
        return false;
        exit(1);
      }
    }

  //clear grasp_pose_vector
  grasp_pose_vector.clear();

  //opening gripper
  if (!this->openGripper()){
    ROS_ERROR("Failed to open Gripper");
    return false;
    exit(1);
  }

  if (!side_grasps){   //rotating gripper to adjust with different orientations (only works with top grasping)
    if (!this->rotateGripper(object_id)){
      ROS_ERROR("Failed to rotate Gripper");
      return false;
      exit(1);
    }
  }

  //cartesian motion downwards
  /*if (!this->executeAxisCartesianMotion(DOWN, GRIPPER_JAW_LENGTH, 'z')){
    ROS_ERROR("Failed to execute downwards cartesian motion");
    return false;
    exit(1);
  }*/

  //get approach motion parameters
  std::string axis;
  bool direction;

  if (!nh.hasParam("/approach_motion_axis")){
    ROS_ERROR("approach_motion_axis parameter not loaded, did you load grasping yaml file ?");
    exit(1);
  }

  nh.getParam("/approach_motion_axis", axis);

  if (!nh.hasParam("/direction_of_approach")){
    ROS_ERROR("direction_of_approach parameter not loaded, did you load grasping yaml file ?");
    exit(1);
  }

  nh.getParam("/direction_of_approach", direction);

  int count = 0.0;
  while (!this->executeAxisCartesianMotion(direction, GRIPPER_JAW_LENGTH, axis.front())){
    this->rotateGripper(M_PI/6);
    count++;
    if (count == 6.0) {
      ROS_ERROR("Failed to execute downwards cartesian motion");
      return false;
      exit(1);
    }
  }

  //add allowed collision matrix
  if (!this->allowObjectCollision(object_id)){
     ROS_ERROR("Failed to allow collision with object ");
     return false;
     exit(1);
 }

  //closing gripper
  if (!this->closeGripper(object_id)){
    ROS_ERROR("Failed to close Gripper");
    return false;
    exit(1);
  }

  //attaching object to gripper
  if (!this->attachCollisionObject(object_id)){
      ROS_ERROR("Failed to attach collision object ");
      return false;
      exit(1);
  }

  //give time for planning scene to process
  ros::Duration(1.0).sleep();

  //cartesian motion upwards (post-grasp position)
  if (!this->executeAxisCartesianMotion(!direction, GRIPPER_JAW_LENGTH, axis.front())){
    ROS_ERROR("Failed to execute upwards cartesian motion");
    return false;
    exit(1);
  }
  visual_tools->deleteAllMarkers();

  return true;
}

bool akit_pick_place::place(std::string object_id){
  ROS_INFO_STREAM("---------- Starting Place Routine ----------");

  //update start state to current state
  akitState = akitGroup->getCurrentState();
  akitGroup->setStartState(*akitState);

  //moving from post-grasp position to pre-place position

  //loop through all grasp poses
  if(!this->planAndExecuteCartesianGoals(grasp_pose_vector, "pre_place")){
    ROS_ERROR("Failed to plan and execute");
    grasp_pose_vector.clear();
    return false;
    exit(1);
  }

  //clear grasp pose vector
  grasp_pose_vector.clear();

  std::string axis;
  bool direction;

  if (!nh.hasParam("/approach_motion_axis")){
    ROS_ERROR("approach_motion_axis parameter not loaded, did you load grasping yaml file ?");
    exit(1);
  }

  nh.getParam("/approach_motion_axis", axis);

  if (!nh.hasParam("/direction_of_approach")){
    ROS_ERROR("direction_of_approach parameter not loaded, did you load grasping yaml file ?");
    exit(1);
  }

  nh.getParam("/direction_of_approach", direction);

  //cartesian motion downwards
  if(!this->executeAxisCartesianMotion(direction, GRIPPER_JAW_LENGTH, axis.front())){ //experiment and change gripper_jaw_length --> drop the object rather than place it
    ROS_ERROR("Failed to execute downwards cartesian motion");
    return false;
    exit(1);
  }

  //detach object from gripper
  if (!this->detachCollisionObject(object_id)){
     ROS_ERROR("Failed to detach collision object ");
     return false;
     exit(1);
   }

  //give time for planning scene to process
  ros::Duration(1.0).sleep();

  //opening gripper
  if(!this->openGripper()){
    ROS_ERROR("Failed to open Gripper");
    return false;
    exit(1);
  }

  //cartesian motion upwards
  if(!this->executeAxisCartesianMotion(!direction, GRIPPER_JAW_LENGTH, axis.front())){
    ROS_ERROR("Failed to execute upwards cartesian motion");
    return false;
    exit(1);
  }
  //delete published trajectory
  visual_tools->deleteAllMarkers();

  //reset allowed collision matrix
  if (!this->resetAllowedCollisionMatrix(object_id)){
      ROS_ERROR("Failed to reset allowed collision matrix. object still not in collision");
      return false;
      exit(1);
  }

  return true;
}

//-----------------------------world interaction methods------------------------------

moveit_msgs::CollisionObject akit_pick_place::addCollisionCylinder(geometry_msgs::Pose cylinder_pose,
                                                                   std::string cylinder_name, double cylinder_height, double cylinder_radius){

  //service objects
  moveit_msgs::ApplyPlanningScene planningSceneSrv_;
  moveit_msgs::PlanningScene planningSceneMsg_;
  moveit_msgs::CollisionObject cylinder;
  cylinder.id = cylinder_name;
  cylinder.header.stamp = ros::Time::now();
  cylinder.header.frame_id = WORLD_FRAME;
  //primitives
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = cylinder_height;
  primitive.dimensions[1] = cylinder_radius;

  cylinder.primitives.push_back(primitive);
  cylinder.primitive_poses.push_back(cylinder_pose);
  cylinder.operation = moveit_msgs::CollisionObject::ADD;

  //calling apply planning scene service
  planningSceneMsg_.world.collision_objects.push_back(cylinder);
  planningSceneMsg_.is_diff = true;
  planningSceneSrv_.request.scene = planningSceneMsg_;
  planning_scene_diff_client_.call(planningSceneSrv_);
  return cylinder;
}

moveit_msgs::CollisionObject akit_pick_place::addCollisionBlock(geometry_msgs::Pose block_pose,
                                                                std::string block_name, double block_size_x, double block_size_y, double block_size_z ){

  //service objects
  moveit_msgs::ApplyPlanningScene planningSceneSrv_;
  moveit_msgs::PlanningScene planningSceneMsg_;
  moveit_msgs::CollisionObject block;
  block.id = block_name;
  block.header.stamp = ros::Time::now();
  block.header.frame_id = WORLD_FRAME;
  //primitives
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = block_size_x;
  primitive.dimensions[1] = block_size_y;
  primitive.dimensions[2] = block_size_z;
  block.primitives.push_back(primitive);
  block.primitive_poses.push_back(block_pose);
  block.operation = moveit_msgs::CollisionObject::ADD;

  //calling apply planning scene service
  planningSceneMsg_.world.collision_objects.push_back(block);
  planningSceneMsg_.is_diff = true;
  planningSceneSrv_.request.scene = planningSceneMsg_;
  planning_scene_diff_client_.call(planningSceneSrv_);

  return block;
}
//instead of position constraints --> no motion in -z direction of world frame
void akit_pick_place::addGround(){
  geometry_msgs::Pose groundPose;
  groundPose.position.x = 0.0;
  groundPose.position.y = 0.0;
  groundPose.position.z = -0.05;
  groundPose.orientation.x = 0.0;
  groundPose.orientation.y = 0.0;
  groundPose.orientation.z = 0.0;
  groundPose.orientation.w = 1.0;
  this->addCollisionBlock(groundPose, "ground", 10.0,10.0,0.1);
  ros::Duration(1.0).sleep();
}

void akit_pick_place::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  //stores feedback
  if(feedback->MOUSE_DOWN){
    interactive_pose = feedback->pose;
    interactive_name = feedback->marker_name;
    //    ROS_INFO_STREAM("object " << interactive_name << " is at x: "
    //                              << interactive_pose.position.x << " y: "
    //                              << interactive_pose.position.y << " z: "
    //                              << interactive_pose.position.z);
  }
}

void akit_pick_place::addInteractiveMarker(geometry_msgs::Pose marker_pose,
                                           std::string marker_name, shape_msgs::SolidPrimitive shape){

  visualization_msgs::Marker i_marker; // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = BASE_LINK;
  int_marker.pose.position = marker_pose.position;
  int_marker.pose.orientation = marker_pose.orientation;
  int_marker.name = marker_name;
  i_marker.color.r = 0.5;
  i_marker.color.g = 0.5;
  i_marker.color.b = 0.5;
  i_marker.color.a = 1.0;

  if (shape.type == shape_msgs::SolidPrimitive::BOX){ //fix for cuboids
    i_marker.type = visualization_msgs::Marker::CUBE;
    i_marker.scale.x = i_marker.scale.y = i_marker.scale.z = shape.dimensions[0] + 0.01; //cannot be same size as collision object, won't return feedback
  } else if (shape.type == shape_msgs::SolidPrimitive::CYLINDER){
    i_marker.type = visualization_msgs::Marker::CYLINDER;
    i_marker.scale.x = i_marker.scale.y = (2 * shape.dimensions[1]) + 0.01;
    i_marker.scale.z = shape.dimensions[0] + 0.01;
  }

  // add the control to the interactive marker
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(i_marker);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_3D;
  int_marker.controls.push_back(control);

  //add the interactive marker to our collection & tell the server to call processFeedback() when feedback arrives for it
  server->insert(int_marker);
  server->setCallback(int_marker.name, processFeedback);
  server->applyChanges();
}

void akit_pick_place::addInteractiveMarkers(){ //server
  ros::Rate rate(0.75); //updates planning scene --> interactive marker position changes after pick
  while(ros::ok())
  {
    collision_objects_map = planningSceneInterface.getObjects(); //return all collision objects in planning scene
    CollisionObjectsMap::iterator it;
    for (it = collision_objects_map.begin(); it != collision_objects_map.end(); ++it){
      if (it->first == "ground")
        continue;  //skip adding an interactive marker to the ground object
      this->addInteractiveMarker(it->second.primitive_poses[0], it->first, it->second.primitives[0]);
    }
    rate.sleep(); // or use ros::spin(); after removing rate&while loop --> but planning scene is not updated
  }
}

bool akit_pick_place::interactive_pick_place(std::vector<geometry_msgs::Pose> place_positions){

  /*
   * takes input a vector of user desired place locations and waits for user to choose an object to pick
   * pick is interactive --> place is predefined
   */

  marker_sub = nh.subscribe("/akit_pick_place/feedback", 10, processFeedback);
  collision_objects_map = planningSceneInterface.getObjects(); //return all collision objects in planning scene
  CollisionObjectsMap::iterator it;
  AttachedCollisionObjectsMap::iterator a_it;
  int count = 0;

  while (ros::ok() && count < place_positions.size()){
    ROS_INFO_STREAM("Please choose object to pick ");
    //wait for user input
    boost::shared_ptr<const visualization_msgs::InteractiveMarkerFeedback> msgReceived =
        ros::topic::waitForMessage<visualization_msgs::InteractiveMarkerFeedback>("/akit_pick_place/feedback");
    if (msgReceived){
      ROS_INFO_STREAM("-------------------Received object to grasp msg-------------------");
      //loop through all collision objects until matching collision object is found
      for (it = collision_objects_map.begin(); it != collision_objects_map.end(); ++it){
        if (interactive_name == it->first){
          if (it->second.primitives[0].type == shape_msgs::SolidPrimitive::CYLINDER){
            this->generateGrasps(interactive_pose, it->second.primitives[0].dimensions[0], it->second.primitives[0].dimensions[1]);
            if(!this->pick(it->first)){
              ROS_ERROR("Failed to pick");
              return false;
              exit(1);
            }
            break;
          } else if (it->second.primitives[0].type == shape_msgs::SolidPrimitive::BOX){
            this->generateGrasps(interactive_pose, it->second.primitives[0].dimensions[0]);
            if (!this->pick(it->first)){
              ROS_ERROR("Failed to pick");
              return false;
              exit(1);
            }
            break;
          }
        } else {
          ROS_INFO_STREAM("No Collision Object Selected!");
        }
      }
    }
    //placing of attached object
    attached_collision_objects_map = planningSceneInterface.getAttachedObjects();
    for(a_it = attached_collision_objects_map.begin();a_it != attached_collision_objects_map.end(); ++a_it){
      if (a_it->second.object.primitives[0].type == shape_msgs::SolidPrimitive::CYLINDER){
        this->generateGrasps(place_positions[count],a_it->second.object.primitives[0].dimensions[0], a_it->second.object.primitives[0].dimensions[1]);
        if(!this->place(a_it->first)){
          ROS_ERROR("Failed to place");
          return false;
          exit(1);
        }
      } else if (a_it->second.object.primitives[0].type == shape_msgs::SolidPrimitive::BOX){
        this->generateGrasps(place_positions[count],a_it->second.object.primitives[0].dimensions[0]);
        if(!this->place(a_it->first)){
          ROS_ERROR("Failed to place");
          return false;
          exit(1);
        }
      }
      count++;
    }
  }
  return true;
}

bool akit_pick_place::attachTool(std::string tool_frame_id){ //change to tool frame id and remove if conditions!

  std::transform(tool_frame_id.begin(), tool_frame_id.end(), tool_frame_id.begin(), ::tolower);

    if (tool_frame_id != GRIPPER_FRAME && tool_frame_id != BUCKET_FRAME){
      ROS_ERROR("Unknown tool, please write correct tool name");
      return false;
      exit(1);
  }


  //parameters
  double quickcoupler_z, quickcoupler_x, distance_above_gripper;

  if (!nh.hasParam("quickcoupler_z")){
     ROS_ERROR("tool exchange parameter (quickcoupler_z) not loaded, did you load initialization yaml file ?");
     return false;
     exit(1);
  }

  nh.getParam("quickcoupler_z", quickcoupler_z);

  if (!nh.hasParam("quickcoupler_x")){
     ROS_ERROR("tool exchange parameter (quickcoupler_x) not loaded, did you load initialization yaml file ?");
     return false;
     exit(1);
  }

  nh.getParam("quickcoupler_x", quickcoupler_x);

  if (!nh.hasParam("distance_above_gripper")){
    ROS_ERROR("tool exchange parameter (distance_above_gripper) not loaded, did you load initialization yaml file ?");
    return false;
    exit(1);
  }

  nh.getParam("distance_above_gripper", distance_above_gripper);

  tf::Quaternion q = tf::createQuaternionFromRPY(0.0,-M_PI/2,0.0); //rotate 90deg around y-axis
  geometry_msgs::PoseStamped initial_pose_tool_frame, initial_pose_base_frame;

  //create initial pose for quickcoupler
  initial_pose_tool_frame.header.frame_id = tool_frame_id;
  initial_pose_tool_frame.pose.position.x = - quickcoupler_z; //translate the quickcoupler so that both locks match
  initial_pose_tool_frame.pose.position.y = 0.0;
  initial_pose_tool_frame.pose.position.z = distance_above_gripper;
  initial_pose_tool_frame.pose.orientation.x = q[0];
  initial_pose_tool_frame.pose.orientation.y = q[1];
  initial_pose_tool_frame.pose.orientation.z = q[2];
  initial_pose_tool_frame.pose.orientation.w = q[3];

  //allow collision between tool group and quickcoupler --> eef parent link
  if (!this->allowToolCollision(tool_frame_id)){
      ROS_ERROR("tool not removed from collision. please load planning group parameters");
      return false;
      exit(1);
  }

  //transform pose from gripper/bucket frame to base link frame
  transform_listener.waitForTransform(BASE_LINK,tool_frame_id, ros::Time::now(), ros::Duration(0.1)); //avoid time difference exception
  transform_listener.transformPose(BASE_LINK,ros::Time(0), initial_pose_tool_frame, tool_frame_id, initial_pose_base_frame);

  //visualize point
  std::vector<geometry_msgs::Pose> visualize_point;
  visualize_point.push_back(initial_pose_tool_frame.pose);
  this->visualizeGrasps(visualize_point, tool_frame_id);

  //motion planning
  std::vector<geometry_msgs::Pose> points;
  points.push_back(initial_pose_base_frame.pose);
  if(!this->planAndExecuteCartesianGoals(points, "initial pose")){
    ROS_ERROR("Failed to plan and execute");
    return false;
    exit(1);
  }

  //promt user
  visual_tools->prompt("proceed ? ");

  //execute cartesian motion in -x axis direction
  this->executeAxisCartesianMotion(false, distance_above_gripper + quickcoupler_x, 'x');

  //get current joint state
  akitState = akitGroup->getCurrentState();
  akitState->copyJointGroupPositions(akitJointModelGroup,akitJointPositions);

  //promt user
  visual_tools->prompt("proceed ? ");

  //update start state to current state
  akitState = akitGroup->getCurrentState();
  akitGroup->setStartState(*akitState);

  //rotate quickcoupler +90deg
  akitJointPositions[4] += M_PI/2;
  akitGroup->setJointValueTarget(akitJointPositions);
  akitSuccess = (akitGroup->plan(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (akitSuccess){
    akitGroup->execute(MotionPlan);
    ROS_INFO_STREAM("Rotation of quickcoupler --> success");
  } else {
    ROS_ERROR_STREAM("Failed to rotate quickcoupler");
    return false;
    exit(1);
  }

  //activate lock --> no control in rviz

  ROS_INFO_STREAM(tool_frame_id << " Attached Successfully");
}


bool akit_pick_place::detachTool(std::string tool_frame_id){

  std::transform(tool_frame_id.begin(), tool_frame_id.end(), tool_frame_id.begin(), ::tolower);

  if (tool_frame_id != GRIPPER_FRAME && tool_frame_id != BUCKET_FRAME){
    ROS_ERROR("Unknown tool, please write correct tool name");
    return false;
    exit(1);
  }

  //variables
  double quickcoupler_x, distance_above_gripper;

  if (!nh.hasParam("quickcoupler_x")){
    ROS_ERROR("tool exchange parameter (quickcoupler_x) not loaded, did you load initialization yaml file ?");
    return false;
    exit(1);
  }

  nh.getParam("quickcoupler_x", quickcoupler_x);

  if (!nh.hasParam("distance_above_gripper")){
    ROS_ERROR("tool exchange parameter (distance_above_gripper) not loaded, did you load initialization yaml file ?");
    return false;
    exit(1);
  }

  nh.getParam("distance_above_gripper", distance_above_gripper);

  //allow collision between tool group and quickcoupler --> eef parent link
  if (!this->allowToolCollision(tool_frame_id)){
    ROS_ERROR("tool not removed from collision. please load planning group parameters");
    return false;
    exit(1);
  }

  //de-activate lock --> no control in rviz

  //update start state to current state
  akitState = akitGroup->getCurrentState();
  akitGroup->setStartState(*akitState);

  akitState->copyJointGroupPositions(akitJointModelGroup,akitJointPositions);

  //rotate quickcoupler -90deg
  akitJointPositions[4] -= M_PI/2;
  akitGroup->setJointValueTarget(akitJointPositions);
  akitSuccess = (akitGroup->plan(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (akitSuccess){
    akitGroup->execute(MotionPlan);
    ROS_INFO_STREAM("Rotation of quickcoupler --> success");
  } else {
    ROS_ERROR_STREAM("Failed to rotate quickcoupler");
    return false;
    exit(1);
  }

  //execute cartesian motion in +x axis direction
  this->executeAxisCartesianMotion(true, distance_above_gripper + quickcoupler_x, 'x');

   ROS_INFO_STREAM(tool_frame_id << " Detached Successfully");
}
