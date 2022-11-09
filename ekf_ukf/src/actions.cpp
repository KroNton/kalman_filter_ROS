#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

#include "ekf_ukf/actions.h"
#include <cmath>	// for fabs, M_PI 

// P(ID) variable
const double g_kp = 0.5;

// steering angle
float get_goal_heading(geometry_msgs::Pose point1, geometry_msgs::Pose point2) {
    // point1: current pose (x,y), point2: goal pose (x,y), returns angle in rad
    float delta_y = point2.position.y - point1.position.y;
    float delta_x = point2.position.x - point1.position.x;
    return atan2(delta_y, delta_x);
}

// Converts yaw angle from Quaternion to Euler format (radians)
float yaw_from_quaternion(geometry_msgs::Pose pose_msg)
{
  double yaw = tf::getYaw(pose_msg.orientation);
  /*
  tf::Quaternion q_orientation;
  quaternionMsgToTF(pose_msg.orientation, q_orientation);
  double roll, pitch, yaw;
  tf::Matrix3x3(q_orientation).getRPY(roll, pitch, yaw);
  */
  return yaw;
}

void go_to_goal(ros::Publisher &direction_pub, geometry_msgs::Pose current, geometry_msgs::Pose goal, float &dist, bool &steering_flip_flag){
  double err_yaw = err_heading_to_goal(current, goal);
	
	geometry_msgs::Twist vel;		
	
	if (fabs(err_yaw) > g_yaw_precision){
    if (!steering_flip_flag)
    {
      vel.angular.z = -g_kp * err_yaw;
    }
    // alternative steering direction if flip_steering_control = true
    else
    {
      vel.angular.z = g_kp * err_yaw;
    }
  } else {
		vel.angular.z = 0;		
	}
  // set velocity dynamicaly
  float distance_progress = (dist_to_goal(goal, current)/dist);
  // close to start pose
  if(distance_progress > 0.9 ){
    vel.linear.x = 0.15;	
  }
  // half way between start and goal pose
	else if (distance_progress > 0.2 ){
    vel.linear.x = 0.3;
  }
  // close to goal pose
  else {
    vel.linear.x = 0.1;
  }
	direction_pub.publish(vel);
}

void turn_to_goal(ros::Publisher &direction_pub, geometry_msgs::Pose current, geometry_msgs::Pose goal, bool &steering_flip_flag){

  double err_yaw = err_heading_to_goal(current, goal);
  ROS_DEBUG_THROTTLE(1, "Turning towards the goal. Current yaw error: [%f]", err_yaw);

	geometry_msgs::Twist vel;		
	
	if (fabs(err_yaw) > g_yaw_precision){
    if (!steering_flip_flag)
    {
      vel.angular.z = -g_kp * err_yaw;
    }
    // alternative steering direction if flip_steering_control = true
    else
    {
      vel.angular.z = g_kp * err_yaw;
    }
  } else {
		vel.angular.z = 0;		
	}
  // pure turn in place, hard coded linear speed '0'
	vel.linear.x = 0.0;		
	direction_pub.publish(vel);
}

float dist_to_goal(geometry_msgs::Pose point1, geometry_msgs::Pose point2) {
    float delta_x = point1.position.x - point2.position.x;
    float delta_y = point1.position.y - point2.position.y;
    float delta_z = point1.position.z - point2.position.z;
    return sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));
}

double correctAngle(double angle){
    if(angle > M_PI){
        angle = angle - 2*M_PI;
    }
    else if(angle < -M_PI){
        angle= angle + 2*M_PI;
    }
    return angle;
}

double err_heading_to_goal(geometry_msgs::Pose current, geometry_msgs::Pose goal)
{
  double desired_yaw, err_yaw;
  desired_yaw = get_goal_heading(current, goal);
  err_yaw = desired_yaw - yaw_from_quaternion(current);

  return correctAngle(err_yaw);
}

void stop(ros::Publisher &direction_pub){
	geometry_msgs::Twist vel;
	
	vel.linear.x = 0;
	vel.angular.z = 0;
	
	direction_pub.publish(vel);	
}
