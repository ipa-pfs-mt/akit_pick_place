#include <akit_pick_place/akit_pick_place.h>

void akit_pick_place::broadcastFrame(geometry_msgs::PoseStamped pose, std::string frame_id)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  while (ros::ok())
  {
    transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    tf::Quaternion q;
    q.setX(pose.pose.orientation.x);
    q.setY(pose.pose.orientation.y);
    q.setZ(pose.pose.orientation.z);
    q.setW(pose.pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), pose.header.frame_id, frame_id));  // block is the child frame
    rate.sleep();
  }
}

tf::Quaternion akit_pick_place::rotateX(geometry_msgs::PoseStamped pose, double angle)
{
  // convert to rotation matrix
  tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  tf::Matrix3x3 m(q);

  // create 3*3 rotation matrix around z axis of block frame
  tf::Matrix3x3 rotation_matrix_x;
  rotation_matrix_x.setRPY(angle, 0.0, 0.0);

  // rotate the orientation
  tf::Matrix3x3 output_rotation_matrix = rotation_matrix_x * m;

  // convert to quaternion
  tf::Quaternion nq;
  output_rotation_matrix.getRotation(nq);

  return nq;
}

tf::Quaternion akit_pick_place::rotateY(geometry_msgs::PoseStamped pose, double angle)
{
  // convert to rotation matrix
  tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  tf::Matrix3x3 m(q);

  // create 3*3 rotation matrix around z axis of block frame
  tf::Matrix3x3 rotation_matrix_y;
  rotation_matrix_y.setRPY(0.0, angle, 0.0);

  // rotate the orientation
  tf::Matrix3x3 output_rotation_matrix = rotation_matrix_y * m;

  // convert to quaternion
  tf::Quaternion nq;
  output_rotation_matrix.getRotation(nq);

  return nq;
}

tf::Quaternion akit_pick_place::rotateZ(geometry_msgs::PoseStamped pose, double angle)
{
  // convert to rotation matrix
  tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  tf::Matrix3x3 m(q);

  // create 3*3 rotation matrix around z axis of block frame
  tf::Matrix3x3 rotation_matrix_z;
  rotation_matrix_z.setRPY(0.0, 0.0, angle);

  // rotate the orientation
  tf::Matrix3x3 output_rotation_matrix = rotation_matrix_z * m;

  // convert to quaternion
  tf::Quaternion nq;
  output_rotation_matrix.getRotation(nq);

  return nq;
}

std::vector<geometry_msgs::PoseStamped> akit_pick_place::generateGrasps(std::string object_id, bool visualize_grasps)
{
  // get pregrasp shape
  std::vector<double> eef_pregrasp_orientation;

  if (!nh.hasParam("/eef_parent_link_pregrasp_orientation"))
  {
    ROS_ERROR("eef_parent_link_pregrasp_orientation parameter not loaded, did you load grasping parameters .yaml file "
              "?");
    exit(1);
  }

  nh.getParam("/eef_parent_link_pregrasp_orientation", eef_pregrasp_orientation);

  // get collision object from planning scene
  std::vector<std::string> object_id_(1, object_id);
  std::map<std::string, moveit_msgs::CollisionObject> object = planningSceneInterface.getObjects(object_id_);
  std::map<std::string, moveit_msgs::CollisionObject>::iterator object_it = object.find(object_id);

  double p = 0.65;  // grasp starts near edge of object
  double max_open_length;

  // get max open length of gripper
  if (!nh.hasParam("/max_open_length"))
  {
    ROS_WARN("max_open_length parameter (distance between gripper jaws when fully opened) "
             "not loaded, did you load grasping parameters .yaml file ?");
  }

  nh.getParam("/max_open_length", max_open_length);

  // start grasp generation in XZ plane

  // first grasp point
  geometry_msgs::PoseStamped graspXZ, grasp_xz;

  grasp_xz.header.frame_id = object_it->second.id;
  grasp_xz.pose.position.x = -(object_it->second.primitives[0].dimensions[0] / 2) - GRIPPER_LENGTH;
  grasp_xz.pose.position.y = 0.0;
  grasp_xz.pose.position.z = 0.0;
  grasp_xz.pose.orientation.x = eef_pregrasp_orientation[0];
  grasp_xz.pose.orientation.y = eef_pregrasp_orientation[1];
  grasp_xz.pose.orientation.z = eef_pregrasp_orientation[2];
  grasp_xz.pose.orientation.w = eef_pregrasp_orientation[3];

  graspXZ = grasp_xz;

  if (object_it->second.primitives[0].dimensions[1] <= max_open_length)
  {
    for (double i = -(object_it->second.primitives[0].dimensions[2] / 2) * p;
         i <= (object_it->second.primitives[0].dimensions[2] / 2) * p;
         i += (object_it->second.primitives[0].dimensions[2] / 10) * p)
    {
      graspXZ.pose.position.z = i;
      grasps.push_back(graspXZ);
    }
  }

  // start rotating around z-axis os the object
  double angle = M_PI / 2;
  tf::Quaternion w = rotateZ(graspXZ, angle);

  // start grasp generation in YZ plane
  geometry_msgs::PoseStamped graspYZ;
  graspYZ.header.frame_id = object_it->second.id;
  graspYZ.pose.position.x = 0.0;
  graspYZ.pose.position.y = -(object_it->second.primitives[0].dimensions[0] / 2) - GRIPPER_LENGTH;
  graspYZ.pose.position.z = 0.0;
  graspYZ.pose.orientation.x = w.getX();
  graspYZ.pose.orientation.y = w.getY();
  graspYZ.pose.orientation.z = w.getZ();
  graspYZ.pose.orientation.w = w.getW();

  if (object_it->second.primitives[0].dimensions[0] <= max_open_length)
  {
    for (double i = -(object_it->second.primitives[0].dimensions[2] / 2) * p;
         i <= (object_it->second.primitives[0].dimensions[2] / 2) * p;
         i += (object_it->second.primitives[0].dimensions[2] / 10) * p)
    {
      graspYZ.pose.position.z = i;
      grasps.push_back(graspYZ);
    }
  }

  // rotate again around z-axis
  tf::Quaternion e = rotateZ(graspYZ, angle);
  graspXZ.pose.position.x *= -1;
  graspXZ.pose.orientation.x = e.getX();
  graspXZ.pose.orientation.y = e.getY();
  graspXZ.pose.orientation.z = e.getZ();
  graspXZ.pose.orientation.w = e.getW();

  if (object_it->second.primitives[0].dimensions[1] <= max_open_length)
  {
    for (double i = -(object_it->second.primitives[0].dimensions[2] / 2) * p;
         i <= (object_it->second.primitives[0].dimensions[2] / 2) * p;
         i += (object_it->second.primitives[0].dimensions[2] / 10) * p)
    {
      graspXZ.pose.position.z = i;
      grasps.push_back(graspXZ);
    }
  }

  // rotate again around z-axis
  tf::Quaternion r = rotateZ(graspXZ, angle);
  graspYZ.pose.position.y *= -1;
  graspYZ.pose.orientation.x = r.getX();
  graspYZ.pose.orientation.y = r.getY();
  graspYZ.pose.orientation.z = r.getZ();
  graspYZ.pose.orientation.w = r.getW();

  if (object_it->second.primitives[0].dimensions[0] <= max_open_length)
  {
    for (double i = -(object_it->second.primitives[0].dimensions[2] / 2) * p;
         i <= (object_it->second.primitives[0].dimensions[2] / 2) * p;
         i += (object_it->second.primitives[0].dimensions[2] / 10) * p)
    {
      graspYZ.pose.position.z = i;
      grasps.push_back(graspYZ);
    }
  }

  // rotate first grasp point around y-axis of the object
  tf::Quaternion t = rotateY(grasp_xz, angle);

  geometry_msgs::PoseStamped graspXY;
  graspXY.header.frame_id = object_it->second.id;
  graspXY.pose.position.y = 0.0;
  graspXY.pose.position.z = (object_it->second.primitives[0].dimensions[2] / 2) + GRIPPER_LENGTH;
  graspXY.pose.orientation.x = t.getX();
  graspXY.pose.orientation.y = t.getY();
  graspXY.pose.orientation.z = t.getZ();
  graspXY.pose.orientation.w = t.getW();

  if (object_it->second.primitives[0].dimensions[1] <= max_open_length)
  {
    for (double i = -(object_it->second.primitives[0].dimensions[0] / 2) * p;
         i <= (object_it->second.primitives[0].dimensions[0] / 2) * p;
         i += (object_it->second.primitives[0].dimensions[0] / 10) * p)
    {
      graspXY.pose.position.x = i;
      grasps.push_back(graspXY);
    }
  }

  // rotate around z axis
  tf::Quaternion y = rotateZ(graspXY, angle);
  graspXY.pose.position.x = 0.0;
  graspXY.pose.orientation.x = y.getX();
  graspXY.pose.orientation.y = y.getY();
  graspXY.pose.orientation.z = y.getZ();
  graspXY.pose.orientation.w = y.getW();

  if (object_it->second.primitives[0].dimensions[0] <= max_open_length)
  {
    for (double i = -(object_it->second.primitives[0].dimensions[1] / 2) * p;
         i <= (object_it->second.primitives[0].dimensions[1] / 2) * p;
         i += (object_it->second.primitives[0].dimensions[1] / 10) * p)
    {
      graspXY.pose.position.y = i;
      grasps.push_back(graspXY);
    }
  }

  // flip
  angle *= -1;
  tf::Quaternion u = rotateY(grasp_xz, angle);

  graspXY.header.frame_id = object_it->second.id;
  graspXY.pose.position.y = 0.0;
  graspXY.pose.position.z = -(object_it->second.primitives[0].dimensions[2] / 2) - GRIPPER_LENGTH;
  graspXY.pose.orientation.x = u.getX();
  graspXY.pose.orientation.y = u.getY();
  graspXY.pose.orientation.z = u.getZ();
  graspXY.pose.orientation.w = u.getW();

  if (object_it->second.primitives[0].dimensions[1] <= max_open_length)
  {
    for (double i = -(object_it->second.primitives[0].dimensions[0] / 2) * p;
         i <= (object_it->second.primitives[0].dimensions[0] / 2) * p;
         i += (object_it->second.primitives[0].dimensions[0] / 10) * p)
    {
      graspXY.pose.position.x = i;
      grasps.push_back(graspXY);
    }
  }

  // rotate around z axis
  tf::Quaternion o = rotateZ(graspXY, angle);
  graspXY.pose.position.x = 0.0;
  graspXY.pose.orientation.x = o.getX();
  graspXY.pose.orientation.y = o.getY();
  graspXY.pose.orientation.z = o.getZ();
  graspXY.pose.orientation.w = o.getW();

  if (object_it->second.primitives[0].dimensions[0] <= max_open_length)
  {
    for (double i = -(object_it->second.primitives[0].dimensions[1] / 2) * p;
         i <= (object_it->second.primitives[0].dimensions[1] / 2) * p;
         i += (object_it->second.primitives[0].dimensions[1] / 10) * p)
    {
      graspXY.pose.position.y = i;
      grasps.push_back(graspXY);
    }
  }

  if (visualize_grasps)
  {
    sleep(1.0);
    this->visualizeGraspPose(grasps);
  }

  return grasps;
}

// transforms grasps to base frame for motion planning
void akit_pick_place::transformGrasps(std::vector<geometry_msgs::PoseStamped>& grasps)
{
  // std::vector<geometry_msgs::PoseStamped> transformed_grasps(grasps.begin(), grasps.end());
  for (std::size_t i = 0; i < grasps.size(); ++i)
  {
    // transform from object frame to base frame, wait to avoid time difference exceptions
    transform_listener.waitForTransform(BASE_LINK, grasps[0].header.frame_id, ros::Time::now(), ros::Duration(0.1));
    transform_listener.transformPose(BASE_LINK, ros::Time(0), grasps[i], grasps[0].header.frame_id, grasps[i]);
  }
}

void akit_pick_place::visualizeGraspPose(std::vector<geometry_msgs::PoseStamped>& grasps)
{
  geometry_msgs::PoseArray poseArray;
  poseArray.header.frame_id = grasps[0].header.frame_id;

  for (int i = 0; i < grasps.size(); i++)
  {
    poseArray.poses.push_back(grasps[i].pose);
  }
  pose_pub.publish(poseArray);
}

void akit_pick_place::scoreGrasps(std::vector<geometry_msgs::PoseStamped>& grasps)
{
  // calculate euclidean distance between each grasp pose and the object frame origin

  // object frame origin is 0 0 0, grasps in object frame

  std::multimap<double, geometry_msgs::PoseStamped> score_map;  // sorted container with multi key values
  std::multimap<double, geometry_msgs::PoseStamped>::iterator it;

  for (std::size_t i = 0; i < grasps.size(); ++i)
  {
    double euclidean_distance =
        std::sqrt(std::pow(grasps[i].pose.position.x, 2) + std::pow(grasps[i].pose.position.y, 2) +
                  std::pow(grasps[i].pose.position.z, 2));
    score_map.insert({ euclidean_distance, grasps[i] });
  }

  grasps.clear();

  for (it = score_map.begin(); it != score_map.end(); ++it)
  {
    grasps.push_back(it->second);
  }
}
