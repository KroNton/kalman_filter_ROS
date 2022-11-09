#ifndef WAYPOINTS_H
#define WAYPOINTS__H

tf::Quaternion RPYToQuaternion(float R, float P, float Y);
std::vector<geometry_msgs::Pose> build_waypoint_list();

#endif