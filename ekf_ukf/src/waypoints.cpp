#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>

#include <vector>

// Function who transforms Euler agles (RPY) in quaternion
tf::Quaternion RPYToQuaternion(float R, float P, float Y)
{
  tf::Matrix3x3 mat;
  mat.setEulerYPR(Y,P,R);

  tf::Quaternion quat;
  mat.getRotation(quat);

  return quat;
}

// attention: hard coded lenght of waypoint list = 4 (see main.cpp)
std::vector<geometry_msgs::Pose> build_waypoint_list(){
  std::vector<geometry_msgs::Pose> waypoint_list;
  geometry_msgs::Pose target_waypoint;
  tf::Quaternion qt;

  qt = RPYToQuaternion(0, 0, 1.57);
  target_waypoint.orientation.w = qt.getW();
  target_waypoint.orientation.x = qt.getX();
  target_waypoint.orientation.y = qt.getY();
  target_waypoint.orientation.z = qt.getZ();
  target_waypoint.position.x = 3.0;
  target_waypoint.position.y = 0.0;
  target_waypoint.position.z = 0.0;
  waypoint_list.push_back(target_waypoint);

  qt = RPYToQuaternion(0, 0, -0.75);
  target_waypoint.orientation.w = qt.getW();
  target_waypoint.orientation.x = qt.getX();
  target_waypoint.orientation.y = qt.getY();
  target_waypoint.orientation.z = qt.getZ();
  target_waypoint.position.x = 3.0;
  target_waypoint.position.y = 3.0;
  target_waypoint.position.z = 0.0;
  waypoint_list.push_back(target_waypoint);

  qt = RPYToQuaternion(0, 0, 0.85);
  target_waypoint.orientation.w = qt.getW();
  target_waypoint.orientation.x = qt.getX();
  target_waypoint.orientation.y = qt.getY();
  target_waypoint.orientation.z = qt.getZ();
  target_waypoint.position.x = 0.0;
  target_waypoint.position.y = 3.0;
  target_waypoint.position.z = 0.0;
  waypoint_list.push_back(target_waypoint);

  qt = RPYToQuaternion(0, 0, 0);
  target_waypoint.orientation.w = qt.getW();
  target_waypoint.orientation.x = qt.getX();
  target_waypoint.orientation.y = qt.getY();
  target_waypoint.orientation.z = qt.getZ();
  target_waypoint.position.x = 0.0;
  target_waypoint.position.y = 0.0;
  target_waypoint.position.z = 0.0;
  waypoint_list.push_back(target_waypoint);

return waypoint_list;
}