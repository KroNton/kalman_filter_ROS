#!/usr/bin/env python

"""
  @author Roberto Zegers
  @brief Spawns or teleports Gazebo model to new pose
         depending on if the model exists or not
  @date Sept 8, 2020
  @license License BSD-3-Clause
  @copyright Copyright (c) 2020, Roberto Zegers
"""

import rospy
import rospkg
import tf
from gazebo_msgs.srv import SpawnModel, GetWorldProperties, DeleteModel
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

class RespawnModel:

  def __init__(self):
      self.package_name = self.get_param("~package_name","particle_filter")
      self.relative_path = self.get_param("~relative_path","/urdf/turtlebot2.urdf")
      self.model_name = self.get_param("~model_name_in_simulation","turtlebot2")
      self.init_pose_x = self.get_param("~initial_pose_x",0.0)
      self.init_pose_y = self.get_param("~initial_pose_y",0.0)
      self.init_pose_a = self.get_param("~initial_pose_a",0.0)

  def get_param(self, name, default):
      """
      Gets one input parameter from the ROS parameter server. 
      If not found, a warning is printed.
      @type name: string
      @param name: Parameter name
      @param default: Default value for the parameter. The type should be 
      the same as the one expected for the parameter.
      @return: The resulting parameter
      """
      if not rospy.has_param(name):
        rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
      return rospy.get_param(name, default)
      
  def delete_gazebo_model(self,model_name):
      """
      Removes a model from Gazebo if the model name exists
      @return None
      """
      rospy.wait_for_service('gazebo/get_world_properties', timeout=10)
      get_world_properties_proxy = rospy.ServiceProxy('gazebo/get_world_properties',
                                                      GetWorldProperties)
      world_properties = get_world_properties_proxy()
      get_world_properties_proxy.close()

      rospy.wait_for_service('gazebo/delete_model', timeout=10)
      delete_model_proxy = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
      if model_name in world_properties.model_names:
          rospy.loginfo("Cleaning model " + model_name)
          delete_model_proxy(model_name)
      delete_model_proxy.close()

  def spawn_model(self, pkg_name, relative_path_urdf, name_in_simulation, model_pose):
      """ 
      Spawn URDF model to pose indicated by Pose() msg
      @param name of the ros package containing the model
      @param relative path inside the package, including file extension .sdf
      @param name of the model as it has to apear in Gazebo
      @param ROS Pose() message containing at least x and y coordinates
      @return None
      """
      try:
          model_path = rospkg.RosPack().get_path(pkg_name)
          file_xml = open(model_path + relative_path_urdf, 'r')
          model_xml = file_xml.read()
      except IOError as err:
          rospy.logerr("Cannot find model [%s], check model name and that model exists, I/O error message:  %s"%(model_object.name, err))
      except UnboundLocalError as error:
          rospy.logdebug("Cannot find package [%s], check package name and that package exists, error message:  %s"%(package_name, error))

      try:
          rospy.logdebug("Waiting for service gazebo/spawn_urdf_model")
          # block max. 5 seconds until the service is available
          rospy.wait_for_service('gazebo/spawn_urdf_model',5.0)
          spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
          # use service handle just like a normal function and call it
          spawn_model_prox(name_in_simulation, model_xml, '', model_pose, 'world')
      except (rospy.ServiceException, rospy.ROSException), e:
          rospy.logerr("Service call failed: %s" % (e,))

  def move_model(self, model_name, new_pose):
      """ Teleports existent model to a given pose
          @param model_name name of the model, must match the name of the model in the (Gazebo) world
          @param new_pose ROS Pose() object
          @return None
      """
      rospy.logdebug("Wait for service 'gazebo/set_model_state'")
      rospy.wait_for_service('gazebo/set_model_state', timeout=5)
      set_model_state = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
      # Define new pose (teleportation target pose)
      state_msg = ModelState()
      state_msg.model_name = model_name
      state_msg.reference_frame = 'map'
      state_msg.pose = new_pose
      try:
          set_model_state(state_msg)
      except rospy.ServiceException, e:
          rospy.logerr("Service call to 'gazebo/set_model_state' failed: {}".format(e))

  def list_to_pose(self, pose_list):
      """ Transforms a list into a Pose() type
          converts euler to quaternion when appropriate
          @param pose_list list with either 6 or 7 elements (x,y,z,r,p,y) or (x,y,z,qx,qy,qz,qw)
          @ return Pose() object
      """
      pose_msg = Pose()
      if len(pose_list) == 7:
          pose_msg.position.x = pose_list[0]
          pose_msg.position.y = pose_list[1]
          pose_msg.position.z = pose_list[2]
          pose_msg.orientation.x = pose_list[3]
          pose_msg.orientation.y = pose_list[4]
          pose_msg.orientation.z = pose_list[5]
          pose_msg.orientation.w = pose_list[6]
      elif len(pose_list) == 6:
          pose_msg.position.x = pose_list[0]
          pose_msg.position.y = pose_list[1]
          pose_msg.position.z = pose_list[2]
          q = tf.transformations.quaternion_from_euler(pose_list[3], pose_list[4], pose_list[5])
          pose_msg.orientation.x = q[0]
          pose_msg.orientation.y = q[1]
          pose_msg.orientation.z = q[2]
          pose_msg.orientation.w = q[3]
      else:
          rospy.logerr("Expected either 6 or 7 elements in list: (x,y,z,r,p,y) or (x,y,z,qx,qy,qz,qw)")
      return pose_msg

  def detect_model(self, model_name):
      """ Verifies if 'model_name' exists in Gazebo
          @param model_name the name of the model in the (Gazebo) world
          @return True or False
      """
      rospy.wait_for_service('gazebo/get_world_properties', timeout=10)
      get_world_properties_proxy = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)
      try:
        world_properties = get_world_properties_proxy()
        get_world_properties_proxy.close()
        if model_name in world_properties.model_names:
          rospy.logdebug("Model detected: %s", model_name)
          return True
        else:
          return False
      except rospy.ServiceException, e:
        rospy.logerr("Service call to '/gazebo/get_world_properties' failed: {}".format(e))
        return False

  def run(self):
    if self.detect_model(self.model_name):
      self.move_model(self.model_name, self.list_to_pose([self.init_pose_x, self.init_pose_y, 0, 0, 0, self.init_pose_a]))
    else:
      self.spawn_model(self.package_name, self.relative_path, self.model_name, self.list_to_pose([self.init_pose_x, self.init_pose_y, 0, 0, 0, self.init_pose_a]))

if __name__ == '__main__':
  rospy.init_node('respawn_node', anonymous=False, log_level=rospy.INFO)
  respawn_model = RespawnModel()
  respawn_model.run()
    