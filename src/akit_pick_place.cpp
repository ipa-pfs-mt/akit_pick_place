#include <akit_pick_place/akit_pick_place.h>

geometry_msgs::Pose akit_pick_place::interactive_pose;
std::string akit_pick_place::interactive_name;

akit_pick_place::akit_pick_place(std::string planning_group_, std::string eef_group_,
                                 std::string base_link_, std::string eef_parent_link_, double gripper_length_,
                                 double gripper_jaw_length_, double gripper_side_length_, bool set_from_grasp_generator_){
  PLANNING_GROUP_NAME = planning_group_;
  EEF_GROUP = eef_group_;
  BASE_LINK = base_link_;
  GRIPPER_LENGTH = gripper_length_;
  GRIPPER_JAW_LENGTH = gripper_jaw_length_;
  GRIPPER_SIDE_LENGTH = gripper_side_length_;
  EEF_PARENT_LINK = eef_parent_link_;
  setFromGraspGenerator = set_from_grasp_generator_;
  waypoints = std::vector<geometry_msgs::Pose>(1);
  akitGroup = new moveit::planning_interface::MoveGroupInterface(planning_group_);
  gripperGroup = new moveit::planning_interface::MoveGroupInterface(eef_group_);
  akitJointModelGroup = akitGroup->getCurrentState()->getJointModelGroup(planning_group_);
  gripperJointModelGroup = gripperGroup->getCurrentState()->getJointModelGroup(eef_group_);
  gripperState = gripperGroup->getCurrentState();
  gripperState->copyJointGroupPositions(gripperJointModelGroup,gripperJointPositions);
  server.reset(new interactive_markers::InteractiveMarkerServer("akit_pick_place","",false));
  visual_tools.reset(new moveit_visual_tools::MoveItVisualTools(base_link_, "visualization_marker"));
}

akit_pick_place::akit_pick_place(){
  PLANNING_GROUP_NAME = "e1_complete";
  EEF_GROUP = "gripper";
  BASE_LINK = "chassis";
  GRIPPER_LENGTH = 1.05;
  GRIPPER_JAW_LENGTH = 0.30;
  GRIPPER_SIDE_LENGTH = 0.20;
  EEF_PARENT_LINK = "quickcoupler";
  setFromGraspGenerator = true;
  waypoints = std::vector<geometry_msgs::Pose>(1);
  akitGroup = new moveit::planning_interface::MoveGroupInterface("e1_complete");
  gripperGroup = new moveit::planning_interface::MoveGroupInterface("gripper");
  akitJointModelGroup = akitGroup->getCurrentState()->getJointModelGroup("e1_complete");
  gripperJointModelGroup = gripperGroup->getCurrentState()->getJointModelGroup("gripper");
  gripperState = gripperGroup->getCurrentState();
  gripperState->copyJointGroupPositions(gripperJointModelGroup,gripperJointPositions);
  server.reset(new interactive_markers::InteractiveMarkerServer("akit_pick_place","",false));
  visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("chassis", "visualization_marker"));
}
akit_pick_place::~akit_pick_place(){
}
void akit_pick_place::setBaseLink(std::string base_link_){
  BASE_LINK = base_link_;
}
void akit_pick_place::setDefaultPlanningGroup(){
  PLANNING_GROUP_NAME = "e1_complete";
}
void akit_pick_place::setGripperGroup(std::string eef_group_){
  EEF_GROUP = eef_group_;
}
void akit_pick_place::setPlanningGroup(std::string planning_group_){
  PLANNING_GROUP_NAME = planning_group_;
}
void akit_pick_place::setPreGraspPose(geometry_msgs::Pose preGraspPose){
  pre_grasp_pose = preGraspPose;
}
void akit_pick_place::setPrePlacePose(geometry_msgs::Pose prePlacePose){
  pre_place_pose = prePlacePose;
}
std::string akit_pick_place::getPlanningGroup(){
  return PLANNING_GROUP_NAME;
}
std::string akit_pick_place::getGripperGroup(){
  return EEF_GROUP;
}
std::string akit_pick_place::getBaseLink(){
  return BASE_LINK;
}
void akit_pick_place::displayTrajectory(moveit::planning_interface::MoveGroupInterface::Plan motion_plan_trajectory,
                                               geometry_msgs::Pose published_pose_frame, std::string axis_name,
                                               rviz_visual_tools::colors color){

  visual_tools->publishAxisLabeled(published_pose_frame, axis_name , rviz_visual_tools::scales::LARGE);
  visual_tools->publishTrajectoryLine(motion_plan_trajectory.trajectory_, akitJointModelGroup,color);
  visual_tools->trigger();
}

bool akit_pick_place::generateGrasps(geometry_msgs::Pose block_pose_, double block_size_, bool visualize){

  //create yaw angle (rotation around z-axis)
  double yaw = atan2(block_pose_.position.y,block_pose_.position.x);

  //calculate length between base frame origin to object frame
  double line_length = sqrt(pow(block_pose_.position.x,2)+pow(block_pose_.position.y,2));
  double number_of_steps = 10.0;

  //grasp distance covered = length of block hypotenuse + 2*gripper side length
  double blockHypotenuse = sqrt(pow(block_size_,2)+pow(block_size_,2)); //improve for all positions!!
  double covered_distance = blockHypotenuse + (2 * GRIPPER_SIDE_LENGTH);
  double starting_point = line_length - (blockHypotenuse/2) - GRIPPER_SIDE_LENGTH;
  double step_size = covered_distance / number_of_steps;

  //grasp_pose_vector = std::vector<geometry_msgs::Pose>(number_of_steps); //initialize
  tf::Quaternion q = tf::createQuaternionFromRPY(0.0,0.0,yaw); //fix rotation to be only around z-axis
                                                               /*y-axis tested but gives bad results*/
  for (double i = step_size; i <= covered_distance; i += step_size){
       grasp_pose.position.x = (starting_point + i) * cos(yaw);
       grasp_pose.position.y = (starting_point + i) * sin(yaw);
       grasp_pose.position.z = GRIPPER_LENGTH + block_pose_.position.z + (block_size_/2); //divide by 2 --> center of gravity of cube
       grasp_pose.orientation.x = q[0];
       grasp_pose.orientation.y = q[1];
       grasp_pose.orientation.z = q[2];
       grasp_pose.orientation.w = q[3];
       grasp_pose_vector.push_back(grasp_pose);
     }

   //visualization of grasp points
  if(visualize){
    this->visualizeGrasps();
  }
  return true;
}

bool akit_pick_place::generateGrasps(geometry_msgs::Pose cuboid_pose_, double cuboid_x_, double cuboid_y_, double cuboid_z_, bool visualize){

  //create yaw angle (rotation around z-axis)
  double yaw = atan2(cuboid_pose_.position.y,cuboid_pose_.position.x);

  //calculate length between base frame origin to object frame
  double line_length = sqrt(pow(cuboid_pose_.position.x,2)+pow(cuboid_pose_.position.y,2));
  double number_of_steps = 10.0;

  //grasp distance covered = length of cuboid hypotenuse + 2*gripper_side_length
  double cuboidHypotenuse = sqrt(pow(cuboid_x_,2)+pow(cuboid_y_,2));
  double covered_distance = cuboidHypotenuse + (2 * GRIPPER_SIDE_LENGTH);
  double starting_point = line_length - (cuboidHypotenuse/2) - GRIPPER_SIDE_LENGTH;
  double step_size = covered_distance / number_of_steps;

  //grasp_pose_vector = std::vector<geometry_msgs::Pose>(number_of_steps); //initialize
  tf::Quaternion q = tf::createQuaternionFromRPY(0.0,0.0,yaw); //fix rotation to be only around z-axis
                                                               /*y-axis tested but gives bad results*/
  for (double i = step_size; i <= covered_distance; i += step_size){
       grasp_pose.position.x = (starting_point + i) * cos(yaw);
       grasp_pose.position.y = (starting_point + i) * sin(yaw);
       grasp_pose.position.z = GRIPPER_LENGTH + cuboid_pose_.position.z + (cuboid_z_/2); //divide by 2 --> center of gravity of cuboid
       grasp_pose.orientation.x = q[0];
       grasp_pose.orientation.y = q[1];
       grasp_pose.orientation.z = q[2];
       grasp_pose.orientation.w = q[3];
       grasp_pose_vector.push_back(grasp_pose);
     }
  if(visualize){
    this->visualizeGrasps();
  }
  return true;
}

bool akit_pick_place::generateGrasps(geometry_msgs::Pose cylinder_pose_, double cylinder_height_, double cylinder_radius_, bool visualize){
  //create yaw angle (rotation around z-axis)
  double yaw = atan2(cylinder_pose_.position.y,cylinder_pose_.position.x);

  //calculate length between base frame origin to object frame
  double line_length = sqrt(pow(cylinder_pose_.position.x,2)+pow(cylinder_pose_.position.y,2));
  double number_of_steps = 10.0;

  //grasp distance covered = diameter + 2*gripper side length
  double cylinderDiameter = 2 * cylinder_radius_;
  double covered_distance = cylinderDiameter + (2 * GRIPPER_SIDE_LENGTH);
  double starting_point = line_length - cylinder_radius_ - GRIPPER_SIDE_LENGTH;
  double step_size = covered_distance / number_of_steps;

  //grasp_pose_vector = std::vector<geometry_msgs::Pose>(number_of_steps); //initialize
  tf::Quaternion q = tf::createQuaternionFromRPY(0.0,0.0,yaw); //fix rotation to be only around z-axis
                                                               /*y-axis tested but gives bad results*/
  for (double i = step_size; i <= covered_distance; i += step_size){
       grasp_pose.position.x = (starting_point + i) * cos(yaw);
       grasp_pose.position.y = (starting_point + i) * sin(yaw);
       grasp_pose.position.z = GRIPPER_LENGTH + cylinder_pose_.position.z + (cylinder_height_/2); //divide by 2 --> center of gravity of cylinder
       grasp_pose.orientation.x = q[0];
       grasp_pose.orientation.y = q[1];
       grasp_pose.orientation.z = q[2];
       grasp_pose.orientation.w = q[3];
       grasp_pose_vector.push_back(grasp_pose);
     }
  if(visualize){
    this->visualizeGrasps();
  }
  return true;
}

bool akit_pick_place::visualizeGrasps(){
  ROS_INFO_STREAM("---------- *Grasp Points visualization* ----------");
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",10);
  uint32_t shape = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = BASE_LINK;
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
  for (int i = 0; i < grasp_pose_vector.size(); ++i){
     marker.id = i;
     marker.action = visualization_msgs::Marker::ADD;
     marker.pose = grasp_pose_vector[i];
     while (marker_pub.getNumSubscribers() < 1)
         {
           ROS_WARN_ONCE("Please create a subscriber to the marker");
           sleep(1);
         }
      marker_pub.publish(marker);
   }
}
bool akit_pick_place::rotateGripper(bool plan_only){
  gripperJointPositions[0] += 0.2;
  gripperGroup->setJointValueTarget(gripperJointPositions);
  if (plan_only){
    gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("Visualising gripper rotation motion plan: "<< (gripperSuccess ? "Planned" : "FAILED"));
    return (gripperSuccess ? true : false);
  } else {
    gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("Visualising gripper rotation motion plan: " << (gripperSuccess ? "Planned" : "FAILED"));
    gripperSuccess = (gripperGroup->execute(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("Gripper Motion Plan: " << (gripperSuccess ? "Rotated gripper" : "FAILED TO ROTATE GRIPPER"));
    return (gripperSuccess ? true : false);
  }
}
bool akit_pick_place::openGripper(bool plan_only){
  //gripperJointPositions[0] = 0.0;
  gripperJointPositions[1] = 1.0; //fixed
  gripperJointPositions[2] = 1.0;
  gripperGroup->setJointValueTarget(gripperJointPositions);
  if (plan_only){
    gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("Visualising open gripper motion plan: "<< (gripperSuccess ? "Planned" : "FAILED"));
    return (gripperSuccess ? true : false);
  } else {
    gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("Visualising open gripper motion plan: " << (gripperSuccess ? "Planned" : "FAILED"));
    gripperSuccess = (gripperGroup->execute(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("Gripper Motion Plan: " << (gripperSuccess ? "Opened gripper" : "FAILED TO OPEN GRIPPER"));
    return (gripperSuccess ? true : false);
  }
}
bool akit_pick_place::closeGripper(bool plan_only){
  gripperJointPositions[0] = 0.0;
  gripperJointPositions[1] = 0.7; //relate to object size (perception) --> later
  gripperJointPositions[2] = 0.7;
  gripperGroup->setJointValueTarget(gripperJointPositions); //
  int count = 0.0;
  if (plan_only){
    gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    while (!gripperSuccess){
      ROS_INFO_STREAM("Failed to close Gripper --> rotating Gripper");
      this->rotateGripper();
      gripperJointPositions[1] = 0.7;
      gripperJointPositions[2] = 0.7;
      gripperGroup->setJointValueTarget(gripperJointPositions);
      gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      count++;
      if (count == 15){ //number of rotations to return back to original position
        break;
    }
  }
    ROS_INFO_STREAM("Visualising closed gripper motion plan: " << (gripperSuccess ? "Planned" : "FAILED"));
    return (gripperSuccess ? true : false);
  } else {
    gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    gripperSuccess = (gripperGroup->execute(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    while (!gripperSuccess){
      ROS_INFO_STREAM("Failed to close Gripper --> rotating Gripper"); //remove & add to pick function
                                       /*if (!closed) --> move UP --> call rotate --> move DOWN --> close again (loop)*/
      this->openGripper();
      this->rotateGripper();
      gripperJointPositions[1] = 0.7;
      gripperJointPositions[2] = 0.7;
      gripperGroup->setJointValueTarget(gripperJointPositions);
      gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      gripperSuccess = (gripperGroup->execute(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      count++;
      if (count == 15)
        break;
     }
     ROS_INFO_STREAM("Visualising closed gripper motion plan: " << (gripperSuccess ? "Planned" : "FAILED"));
     ROS_INFO_STREAM("Gripper Motion Plan: " << (gripperSuccess ? "Closed gripper" : "FAILED TO CLOSE GRIPPER"));
     return (gripperSuccess ? true : false);
  }
}
bool akit_pick_place::executeCartesianMotion(bool direction){

  //UP = true, DOWN = false

  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  akitGroup->setMaxVelocityScalingFactor(0.1);
  cartesian_pose = akitGroup->getCurrentPose(EEF_PARENT_LINK).pose;
  if (!direction){ //downwards cartesian motion
      cartesian_pose.position.z -= GRIPPER_JAW_LENGTH;
  } else { //upwards cartesian motion
      cartesian_pose.position.z += GRIPPER_JAW_LENGTH;
  }
    waypoints[0] = cartesian_pose;
    double fraction  = akitGroup->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_STREAM("Visualizing Cartesian Motion plan:  " << (fraction * 100.0) <<"%% achieved");
    if (fraction * 100 > 50.0){
      MotionPlan.trajectory_ = trajectory;
      ROS_INFO_STREAM("====== 3. Executing Cartesian Motion ======");
      akitSuccess = (akitGroup->execute(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_STREAM("cartesian motion plan: " << (akitSuccess ? "EXECUTED MOTION PLAN --> cartesian motion" : "FAILED TO EXECUTE CARTESIAN MOTION PLAN"));
      return (akitSuccess ? true : false);
    } else {
      ROS_ERROR("Cannot execute cartesian motion, plan < 50 %%");
      return false;
    }
}
bool akit_pick_place::pick(std::string object_id){
  ROS_INFO_STREAM("---------- *Starting pick routine* ----------");
  //move from home position to pre-grasp position

  int count = 0;
  bool executed;

  if(setFromGraspGenerator){
    for(int i = 0; i < grasp_pose_vector.size(); ++i){
      akitGroup->setPoseTarget(grasp_pose_vector[i]);
      akitSuccess = (akitGroup->plan(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if(!akitSuccess){
        ROS_INFO_STREAM("Motion Planning to Pre-Grasp Position --------");
        count++;
          if (count == grasp_pose_vector.size()){
            ROS_ERROR("Failed to plan to pre-grasp position");
            return false;
            exit(1);
        }
      } else {
        this->displayTrajectory(MotionPlan,grasp_pose_vector[i],"pre_grasp_pose " + std::to_string(count), rviz_visual_tools::colors::LIME_GREEN);
        executed = (akitGroup->execute(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_STREAM("Executing Motion plan: " << (executed ? "Executed" : "FAILED"));
        if (!executed){
          ROS_ERROR("Failed to execute motion plan to pre-grasp position");
          return false;
          exit(1);
        } else {
         break;
        }
      }
    }
  } else { //if grasp poses are entered from blender (remove later if not needed)
    akitGroup->setPoseTarget(pre_grasp_pose); //make setters to bool
    akitSuccess = (akitGroup->plan(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("Motion Planning to Pre-Grasp Position: " << (akitSuccess ? "Planned" : "FAILED"));
    if (!akitSuccess){
      ROS_ERROR("Failed to plan to pre-grasp position");
      return false;
      exit(1);
    } else {
      this->displayTrajectory(MotionPlan,pre_grasp_pose,"pre_grasp_pose",rviz_visual_tools::colors::LIME_GREEN);
      executed = (akitGroup->execute(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_STREAM("Executing Motion plan: " << (executed ? "Executed" : "FAILED"));
      if (!executed){
        ROS_ERROR("Failed to execute motion plan to pre-grasp position");
        return false;
        exit(1);
      }
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

  //cartesian motion downwards
  if (!this->executeCartesianMotion(DOWN)){
    ROS_ERROR("Failed to execute downwards cartesian motion");
    return false;
    exit(1);
  }

  //closing gripper
  if (!this->closeGripper()){
   ROS_ERROR("Failed to close Gripper");
   return false;
   exit(1);
  }

  //attaching object to gripper
  ROS_INFO_STREAM("Attaching object to gripper");
  bool isattached = gripperGroup->attachObject(object_id);
  ros::Duration(2.0).sleep(); //give time for planning scene to process
  ROS_INFO_STREAM("Attaching object to gripper:" << (isattached ? "Attached" : "FAILED"));
  if(!isattached){
    ROS_ERROR("Failed to attach object to gripper");
    return false;
    exit(1);
  }

  //cartesian motion upwards (post-grasp position)
  if (!this->executeCartesianMotion(UP)){
    ROS_ERROR("Failed to execute upwards cartesian motion");
    return false;
    exit(1);
  }
  visual_tools->deleteAllMarkers();
}
bool akit_pick_place::place(std::string object_id){
  ROS_INFO_STREAM("---------- *Starting place routine* ----------");
  //moving from post-grasp position to pre-place position

  int count = 0;
  bool executed;

  if(setFromGraspGenerator){
    for(int i = 0; i < grasp_pose_vector.size(); ++i){
      akitGroup->setPoseTarget(grasp_pose_vector[i]);
      akitSuccess = (akitGroup->plan(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if(!akitSuccess){
        ROS_INFO_STREAM("Motion Planning to Pre-Place Position --------");
        count++;
          if (count == grasp_pose_vector.size()){
            ROS_ERROR("Failed to plan to pre-Place position");
            return false;
            exit(1);
        }
      } else {
        this->displayTrajectory(MotionPlan,grasp_pose_vector[i],"pre_place_pose " + std::to_string(count), rviz_visual_tools::colors::MAGENTA);
        executed = (akitGroup->execute(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_STREAM("Executing Motion plan: " << (executed ? "Executed" : "FAILED"));
        if (!executed){
          ROS_ERROR("Failed to execute motion plan to pre-place position");
          return false;
          exit(1);
        } else {
         break;
        }
      }
    }
  } else { //if place poses are entered from blender (remove later if not needed)
      akitGroup->setPoseTarget(pre_place_pose);
      akitSuccess = (akitGroup->plan(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_STREAM("Motion Planning to Pre-Place Position: " << (akitSuccess ? "Planned" : "FAILED"));
      if (!akitSuccess){
        ROS_ERROR("Failed to plan to pre-place position");
        return false;
        exit(1);
      } else {
        this->displayTrajectory(MotionPlan,pre_place_pose,"pre_place_pose",rviz_visual_tools::colors::MAGENTA);
        executed = (akitGroup->execute(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_STREAM("Executing Motion Plan: " << (executed ? "Executed" : "FAILED"));
        if (!executed){
          ROS_ERROR("Failed to execute motion plan to pre-place position");
          return false;
          exit(1);
        }
      }
  }

  //clear grasp pose vector
  grasp_pose_vector.clear();

  //cartesian motion downwards
  if(!this->executeCartesianMotion(DOWN)){
    ROS_ERROR("Failed to execute downwards cartesian motion");
    return false;
    exit(1);
  }

  //detach object from gripper
  ROS_INFO_STREAM("Detaching object to gripper");
  bool isdetached = gripperGroup->detachObject(object_id);
  ros::Duration(1.0).sleep();
  ROS_INFO_STREAM("Detaching object from gripper: " << (isdetached ? "Detached" : "FAILED"));
  if(!isdetached){
    ROS_ERROR("Failed to attach object to gripper");
    return false;
    exit(1);
  }

  //opening gripper
  if(!this->openGripper()){
    ROS_ERROR("Failed to open Gripper");
    return false;
    exit(1);
  }

  //cartesian motion upwards
  if(!this->executeCartesianMotion(UP)){
    ROS_ERROR("Failed to execute upwards cartesian motion");
    return false;
    exit(1);
  }
  visual_tools->deleteAllMarkers();
}

bool akit_pick_place::pick_place(std::string object_id){ //finalize after testing --> works only with blender (integrate with grasp generator)
  //calling pick method
  if(!this->pick(object_id)){
    ROS_ERROR("Failed to pick");
    return false;
    exit(1);
  }
  //calling place method
  if(!this->place(object_id)){
    ROS_ERROR("Failed to place");
    return false;
    exit(1);
  }
}
//-----------------------------world interaction methods------------------------------

void akit_pick_place::addCollisionCylinder(geometry_msgs::Pose cylinder_pose,
                                              std::string cylinder_name, double cylinder_height, double cylinder_radius){
  collision_objects_vector.clear(); //avoid re-addition of same object
  moveit_msgs::CollisionObject cylinder;
  cylinder.id = cylinder_name;
  cylinder.header.stamp = ros::Time::now();
  cylinder.header.frame_id = BASE_LINK;
  //primitives
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = cylinder_height;
  primitive.dimensions[1] = cylinder_radius;

  cylinder.primitives.push_back(primitive);
  cylinder.primitive_poses.push_back(cylinder_pose);
  cylinder.operation = moveit_msgs::CollisionObject::ADD;

  collision_objects_vector.push_back(cylinder);
  planningSceneInterface.addCollisionObjects(collision_objects_vector);

}

void akit_pick_place::addCollisionBlock(geometry_msgs::Pose block_pose, std::string block_name, double block_size){
    collision_objects_vector.clear(); //avoid re-addition of same object
    moveit_msgs::CollisionObject block;
    block.id = block_name;
    block.header.stamp = ros::Time::now();
    block.header.frame_id = BASE_LINK;
    //primitives
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = primitive.dimensions[1] = primitive.dimensions[2] = block_size;
    block.primitives.push_back(primitive);
    block.primitive_poses.push_back(block_pose);
    block.operation = moveit_msgs::CollisionObject::ADD;

    collision_objects_vector.push_back(block);
    planningSceneInterface.addCollisionObjects(collision_objects_vector);
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

void akit_pick_place::addInteractiveMarker(geometry_msgs::Pose marker_position,
                                           std::string marker_name, shape_msgs::SolidPrimitive shape){

    visualization_msgs::Marker i_marker; // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = BASE_LINK;
    int_marker.pose.position = marker_position.position;
    int_marker.pose.orientation = marker_position.orientation;
    int_marker.name = marker_name;
//    i_marker.color.r = 0.5;
//    i_marker.color.g = 0.5;
//    i_marker.color.b = 0.5;
//    i_marker.color.a = 1.0;

    if (shape.type == shape_msgs::SolidPrimitive::BOX){
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
        this->addInteractiveMarker(it->second.primitive_poses[0], it->first, it->second.primitives[0]);
        }
        rate.sleep(); // or use ros::spin(); after removing rate&while loop --> but planning scene is not updated
    }
 }

bool akit_pick_place::interactive_pick_place(std::vector<geometry_msgs::Pose> place_positions){

  /*
   * takes input a vector of user desired place locations and waits for user to choose
   * which object to pick
   * pick is interactive --> place is predefined
   */

  marker_sub = nh.subscribe("/akit_pick_place/feedback", 10, processFeedback);
  collision_objects_map = planningSceneInterface.getObjects(); //return all collision objects in planning scene
  CollisionObjectsMap::iterator it;
  AttachedCollisionObjectsMap::iterator a_it;
  int count = 0;

  while (ros::ok() && count <= place_positions.size()){
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
             if(!this->pick(interactive_name)){
               ROS_ERROR("Failed to pick");
               return false;
               exit(1);
             }
             break;
          } else if (it->second.primitives[0].type == shape_msgs::SolidPrimitive::BOX){
             this->generateGrasps(interactive_pose, it->second.primitives[0].dimensions[0]);
             if (!this->pick(interactive_name)){
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
}


