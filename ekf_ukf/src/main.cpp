#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>

#include "ekf_ukf/actions.h"
#include "ekf_ukf/waypoints.h"

namespace Robot_States
{
  enum Robot_State
  {
     GO_TO_GOAL,
     TURN_TO_GOAL,
     STOP
  };
}
typedef Robot_States::Robot_State Robot_State;

std::vector<std::string> states_as_text = {"GO TO WAYPOINT", "TURN TO WAYPOINT", "STOP"};

// current robot pose (specified relative to odom)
geometry_msgs::Pose current_pose;

// hard coded goal tolerance
const double g_goal_distance = 0.05;

// hard coded flag, whether to loop waypoints or shutdown node
bool loop = false;

// get odometry
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg){
	current_pose = msg->pose.pose;
  ROS_DEBUG_THROTTLE(2, "I heard (x,y): %f,%f", current_pose.position.x, current_pose.position.y);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "go_to_goal_controller");
	
	// create node
  ros::NodeHandle node;

  // get params
  // change bool value if robot oscillates facing in the opposite direction of target waypoint
  bool flip_steering_control;
  node.param<bool>("flip_steering", flip_steering_control, false);

  // Shutdown service client
  std::string srv_name = "/node_manager/shutdown";
  ros::ServiceClient shutdown_client = node.serviceClient<std_srvs::Empty>(srv_name);
  std_srvs::Empty srv1;
	
	// subscribe to odom 
	ros::Subscriber odom_sub = node.subscribe("odom", 1, odom_callback);
		
	// publish messages on cmd_vel
  ros::Publisher direction_pub;
  direction_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  std::vector<geometry_msgs::Pose> waypoint_list = build_waypoint_list();
  
  // Define goal pose (specified relative to odom)
  geometry_msgs::Pose robot_goal;
  // get first element
  int goal_index = 0;
  robot_goal = waypoint_list[goal_index];
  float original_distance = dist_to_goal(robot_goal, current_pose);

  ROS_INFO("Started waypoint sequence controller node");
  ros::Rate rate(20);
  
  // initialize robot state
	Robot_State diff_drive_state = Robot_States::TURN_TO_GOAL;
  current_pose.orientation.x = 0;
  current_pose.orientation.y = 0;
  current_pose.orientation.z = 0;
  current_pose.orientation.w = 1;
  
  while (ros::ok()){

    ROS_INFO_THROTTLE(4, "Current action: %s", states_as_text[diff_drive_state].c_str());

    switch (diff_drive_state)
    {

    case Robot_States::GO_TO_GOAL:
      if (dist_to_goal(robot_goal, current_pose) < g_goal_distance)
      {
        ROS_INFO("Waypoint reached");
        diff_drive_state = Robot_States::STOP;
      }
      if (fabs(err_heading_to_goal(current_pose, robot_goal)) > g_yaw_precision * 2)
      {
        ROS_DEBUG("Driving forward pointing towards wrong heading. Heading to goal: %f > yaw tolerance: %f ", err_heading_to_goal(current_pose, robot_goal), g_yaw_precision);
        diff_drive_state = Robot_States::TURN_TO_GOAL;
      }
      else
      {
        ROS_DEBUG_THROTTLE(1, "Driving forward pointing towards the goal, distance to goal: %f", dist_to_goal(robot_goal, current_pose));
        go_to_goal(direction_pub, current_pose, robot_goal, original_distance, flip_steering_control);
      }
      break;
    case Robot_States::TURN_TO_GOAL:
      if (fabs(err_heading_to_goal(current_pose, robot_goal)) < g_yaw_precision)
      {
        ROS_DEBUG("Turn to goal has reached the desired heading, %f", err_heading_to_goal(current_pose, robot_goal));
        diff_drive_state = Robot_States::GO_TO_GOAL;
      }
      else
      {
        turn_to_goal(direction_pub, current_pose, robot_goal, flip_steering_control);
      }
      break;
    case Robot_States::STOP:
      stop(direction_pub);
      goal_index++;
      // verify if all waypoints were traversed
      // attention: hard coded lenght of waypoint list = 4
      if (!(goal_index % 4))
      {
        if (loop)
        {
          goal_index = 0;
          robot_goal = waypoint_list[goal_index];
          original_distance = dist_to_goal(robot_goal, current_pose);
          diff_drive_state = Robot_States::TURN_TO_GOAL;
        }
        else
        {
          ROS_INFO("Shuting down move_in_square!");
          // call shutdown service
          shutdown_client.call(srv1);
          ros::shutdown();
        }
      }
      else
      {
        robot_goal = waypoint_list[goal_index];
        original_distance = dist_to_goal(robot_goal, current_pose);
        diff_drive_state = Robot_States::TURN_TO_GOAL;
      }
      break;

    default:
      ROS_INFO("Unknown state!");
      break;
    }

    ros::spinOnce();
		rate.sleep();
  }
  return 0;
}
