#!/usr/bin/env python

"""
  @author  Roberto Zegers
  @brief   Tool for launching, managing and shutting down ROS Node processes programmatically
  @date    Sept 04, 2020
  @license License BSD-3-Clause
  @copyright Copyright (c) 2020, Roberto Zegers
"""

import roslaunch
import rospy
import rospkg
from std_srvs.srv import Empty, EmptyResponse
import os.path

def set_robot_description(package_name,relative_path_filename):
  """
  Loads URDF description to parameter server
  """
  try:
    model_path = rospkg.RosPack().get_path(package_name)
    file_xml = open(model_path + relative_path_filename, 'r')
    model_xml = file_xml.read().replace('\n', '')
    rospy.set_param('/robot_description', model_xml)
    rospy.loginfo("Robot description set!")
  except IOError as err:
    rospy.logerr("Cannot find model [%s], check model name and that model exists, I/O error message:  %s"%(model_object.name,err))
  except UnboundLocalError as error:
    rospy.logdebug("Cannot find package [%s], check package name and that package exists, error message:  %s"%(package_name, error))

def start_launch_file(launch_file):
  """
  Makes use of roslaunch api to start a .launch file
  """
  uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  roslaunch.configure_logging(uuid)
  launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file)
  launch.start()
  return launch

def start_node(package, executable, node_name, arguments):
  """
  Makes use of roslaunch api to launch ROS Nodes
  """
  node = roslaunch.core.Node(package, executable, name=node_name, args=arguments)
  launch = roslaunch.scriptapi.ROSLaunch()
  launch.start()
  process = launch.launch(node)
  return process

def start_process(pkg_name, process_name, node_name=None, arguments=''):
  if (".launch" in process_name):
    launch_file = get_launch_from_pkg(pkg_name, process_name)
    launch_file_arr = []
    launch_file_arr.append(launch_file)
    return start_launch_file(launch_file_arr), "launch"
  else:
    return start_node(pkg_name, process_name, node_name, arguments), "node"

def stop_node(process):
  process.stop()

def stop_launch(launch):
  launch.shutdown()

def stop_process(process, process_type):
  if type == "launch":
    stop_launch(process)
  else:
    stop_node(process)

def get_path_to_pkg(pkg_name):
  rospack = rospkg.RosPack()
  return rospack.get_path(pkg_name)

def get_launch_from_pkg(pkg_name, launch_filename):
  pkg = get_path_to_pkg(pkg_name)
  return os.path.join(pkg, "launch", launch_filename)

def shutdown_callback(_):
  global shutdown_flag
  response = EmptyResponse()
  # set shutdown flag
  shutdown_flag = True
  return response

def clean_shutdown():
  rospy.loginfo("Shutting down node manager...")

if __name__ == '__main__':
  global pub, shutdown_flag
  shutdown_flag = False
  rospy.init_node('node_manager')

  set_robot_description('ekf_ukf','/urdf/turtlebot2.urdf')

  # keeps proccess and process type for all started nodes
  processes = []

  # Spawn a robot into Gazebo 
  processes.extend(start_process('gazebo_ros','spawn_model', arguments="-param robot_description -urdf -z 0 -model mobile_base"))
  processes.extend(start_process('robot_state_publisher','robot_state_publisher', node_name='robot_state_publisher'))
  processes.extend(start_process('ekf_ukf','noisy_odom_republisher.py', node_name='noisy_odom'))
  processes.extend(start_process('ekf_ukf','ekf_filtering.launch'))
  processes.extend(start_process('ekf_ukf','plot_odom_drift.launch'))
  processes.extend(start_process('ekf_ukf','square_path.launch'))

  # shutdown service server
  shutdown_service = rospy.Service('/node_manager/shutdown', Empty, shutdown_callback)
  # cleanup on shutdown signal
  rospy.on_shutdown(clean_shutdown)
  # spin
  while not rospy.core.is_shutdown() and not shutdown_flag:
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)
