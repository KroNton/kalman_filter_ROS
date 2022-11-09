#ifndef GO_TO_GOAL_H
#define GO_TO_GOAL_H

float get_goal_heading(geometry_msgs::Pose point1, geometry_msgs::Pose point2);

float yaw_from_quaternion(geometry_msgs::Pose pose_msg);
void go_to_goal(ros::Publisher &direction_pub, geometry_msgs::Pose current, geometry_msgs::Pose goal, float &dist, bool &steering_flip_flag);

void turn_to_goal(ros::Publisher &direction_pub, geometry_msgs::Pose current, geometry_msgs::Pose goal, bool &steering_flip_flag);
float dist_to_goal(geometry_msgs::Pose point1, geometry_msgs::Pose point2);

double err_heading_to_goal(geometry_msgs::Pose current, geometry_msgs::Pose goal);

void stop(ros::Publisher &direction_pub);

// hard coded heading precision
const double g_yaw_precision 	= 3.14159/90;

#endif