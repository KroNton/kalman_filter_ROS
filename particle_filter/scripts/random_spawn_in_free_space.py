#!/usr/bin/env python

"""
  @author Roberto Zegers
  @brief  Spawns URDF model in Gazebo on a random empty grid cell, considering a square shape robot footprint
  @date   Aug 20, 2020
"""

import rospy
import rospkg
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.srv import GetMap
from gazebo_msgs.srv import SpawnModel, GetWorldProperties, SetModelState
from gazebo_msgs.msg import ModelState
import rosservice

import random
import copy

class SpawnInFreeSpace:

    def __init__(self):
        # in pixel
        self.map_width = 0
        self.map_height = 0
        # the resolution of the map and is expressed in meters/ per pixel
        # or the size of each grid cell or pixel in meters
        # 0.05 means for example 5 centimers for each cell (pixel)
        self.map_resolution = 0
        # the position in grid cell or pixel coordinates of the world coordinate origin
        self.map_origin_x = 0
        self.map_origin_y = 0
        # Pixels with occupancy probability less than this threshold are considered free
        self.free_thresh = 0.196
        # occupancy_grid keeps array of int8 probabilities
        # in the range [0,100]. 0 = free, 100 = obstacle, unknown is -1
        self.occupancy_grid = []
        # list of empty grid cells indices(!)
        self.free_space_grid_cells = []
        self.got_occupancy_grid = False
        self.selected_random_empty_cell_idx = 0
        self.selected_grid_x = 0
        self.selected_grid_y = 0
        # the lenght of one side of a square shape model footprint
        # used to calculate configuration space (value in meters)
        self.side_of_squared_footprint = 0.2
        # occupancy grid array for configuration space
        self.cspace_map = []
        self.package_name = ""
        self.relative_path = ""
        self.model_name = ""
    
    def parse_empty_cells(self, map_array):
        """
        Populates free_space_grid_cells with indices pointing to empty grid cells
        found on an occupancy grid array
        @param occupancy grid array
        @return None
        """
        if self.map_width>0 and self.map_height>0:
            rospy.logdebug("Size of map data array is: %d", len(map_array))
            # create list of empty grid cells indices
            for i in range(self.map_height):
              for j in range(self.map_width):
                if ( (map_array[i*self.map_width + j] < self.free_thresh) and (map_array[i*self.map_width + j] >= 0) ):
                  # append index
                  self.free_space_grid_cells.append(i*self.map_width + j) 
            rospy.logdebug("Number of free space grid cells found: %d", len(self.free_space_grid_cells))

    def generate_cspace_map(self, map_array):
        """
        Traverse the map and update occupancy value of grid cells adjacent to obstacles
        according to a given footprint so that a model does not hit close-by obstacles when spawn
        @param occupancy grid array
        @return None
        """
        # get the number of cells/pixels to be updated on each side of an occupied cell
        inflate_factor = int((self.side_of_squared_footprint / self.map_resolution)/2)

        # keep the original occupancy grid unchanged
        config_space = copy.deepcopy(map_array)
        working_config_space = copy.copy(config_space)
        working_config_space = list(working_config_space)

        # traverse the map to see if close-by grid cells have to be updated
        for row in range(0, self.map_height):
            for col in range(0, self.map_width):
                # Get the occupancy probability value in the current grid cell
                grid_cell_idx = row *  self.map_width + col
                grid_cell_idx_y = grid_cell_idx // self.map_width
                cost = config_space[grid_cell_idx]
                # if the occupancy probability value is non-zero, traverse all close-by grid cells
                if cost > self.free_thresh:
                    # traverse adjacent rows
                    for i in range(row - inflate_factor, row + inflate_factor + 1):
                      # traverse adjacent columns
                        for j in range(col - inflate_factor, col + inflate_factor + 1):
                            nearby_cell_idx = i* self.map_width + j
                            # get row of cell being considered
                            nearby_cell_idx_y = nearby_cell_idx // self.map_width
                            # update only if close-by grid cell is within the map bounds
                            if nearby_cell_idx >= 0 and nearby_cell_idx < ( self.map_height * self.map_width):
                                # update only if row of cell being considered is equal to row being traversed (i)
                                if nearby_cell_idx_y == i:
                                    # Update the working configuration space
                                    working_config_space[nearby_cell_idx] = cost

        self.cspace_map = working_config_space
        rospy.loginfo("Configuration map ready!")

    def select_random_empty_cell(self):        
        self.selected_random_empty_cell_idx = random.choice(self.free_space_grid_cells)
        rospy.logdebug("Index for random empty cell: %d", self.selected_random_empty_cell_idx)
        rospy.logdebug("Free space probability value in that cell is: %d", self.occupancy_grid[self.selected_random_empty_cell_idx])
        self.selected_grid_x, self.selected_grid_y = self.indexToGridCoord(self.selected_random_empty_cell_idx)

    def detect_model(self):
        """ Verifies if 'self.model_name' exists in Gazebo
            @return True or False
        """
        rospy.wait_for_service('gazebo/get_world_properties', timeout=10)
        get_world_properties_proxy = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)
        try:
          world_properties = get_world_properties_proxy()
          get_world_properties_proxy.close()
          if self.model_name in world_properties.model_names:
            rospy.logdebug("Model detected: %s", self.model_name)
            return True
          else:
            return False
        except rospy.ServiceException, e:
          rospy.logerr("Service call to '/gazebo/get_world_properties' failed: {}".format(e))
          return False

    def move_model_random(self):
        """ Sets a new random Pose() and moves existent model
            @return None
        """
         # find a new random empty cell
        self.select_random_empty_cell()
        rospy.loginfo("Wait for service 'gazebo/set_model_state'")
        rospy.wait_for_service('gazebo/set_model_state', timeout=5)
        set_model_state = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

        # Define new pose (teleportation target pose)
        state_msg = ModelState()
        state_msg.model_name = self.model_name
        state_msg.reference_frame = 'map'
        state_msg.pose = self.gridToWorld(self.selected_grid_x,self.selected_grid_y)
        # set identity quaternion
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1

        try:
            set_model_state(state_msg)
        except rospy.ServiceException, e:
            rospy.logerr("Service call to 'gazebo/set_model_state' failed: {}".format(e))

    def spawn_model(self, pkg_name, relative_path_urdf, name_in_simulation, model_pose):
        """ Spawn URDF model as indicated by Pose() msg
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
        except ResourceNotFound as e:
            rospy.logdebug("Cannot find path to package [%s], check package name and that package exists, error message:  %s"%(package_name, e))

        try:
            rospy.logdebug("Waiting for service gazebo/spawn_urdf_model")
            # block max. 5 seconds until the service is available
            rospy.wait_for_service('gazebo/spawn_urdf_model',5.0)
            spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            # use service handle just like a normal function and call it
            spawn_model_prox(name_in_simulation, model_xml, '', model_pose, 'world')
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))

    def indexToGridCoord(self, array_index):
        """
        Converts a linear index value to a x,y grid cell coordinate value
        This transformation is derived from map width
        @param a linear index value, specifying a cell/pixel in an 1-D array
        @return
        """
        grid_cell_map_x = array_index % self.map_width # modulo yields the remainder
        grid_cell_map_y = array_index // self.map_width # floor division '//' yields the quotient
        return grid_cell_map_x, grid_cell_map_y

    def gridToWorld(self, grid_cell_map_x, grid_cell_map_y):
        """
        Convert x,y grid cell/pixel coordinates to world coordinates (in meters)
        This transformation is derived from the map resolution and map origin
        @param grid cell/pixel coordinates
        @return world coordinates as Pose() message
        """
        gazebo_pose = Pose()
        gazebo_pose.position.x = self.map_resolution * grid_cell_map_x + self.map_origin_x
        gazebo_pose.position.y = self.map_resolution * grid_cell_map_y + self.map_origin_y
        gazebo_pose.position.z = 0.0
        return gazebo_pose

    def worldToGrid(self, world_pose):
        """
        Convert Pose() message in world frame of reference to grid map cell/pixel coordinates
        This transformation is derived from the map resolution and map origin
        @param world coordinates as Pose() message (by default in meters)
        @return grid cell/pixel coordinates
        """
        grid_cell_map_x = int((world_pose.position.x + self.map_origin_x) / self.map_resolution)
        grid_cell_map_y = int((world_pose.position.y + self.map_origin_y) / self.map_resolution)
        return grid_cell_map_x, grid_cell_map_y

    def spawn(self):
        # find random empty cell and spawn model
        self.select_random_empty_cell()
        self.spawn_model(self.package_name,self.relative_path,self.model_name,self.gridToWorld(self.selected_grid_x,self.selected_grid_y))
        rospy.loginfo("Spawned model into Gazebo!")

    def init(self):
        rospy.init_node('spawn_in_free_space_node', anonymous=False, log_level=rospy.INFO)

        # retrieve private configuration variables from parameter server
        if rospy.has_param("~package_name"):
            self.package_name = rospy.get_param("~package_name")
        else:
            self.package_name = "particle_filter"
            rospy.logwarn("No '~package_name' parameter found in parameter server, using default value '%s'", package_name)
        if rospy.has_param("~relative_path"):
            self.relative_path = rospy.get_param("~relative_path")
        else:
            self.relative_path = "/urdf/turtlebot2.urdf"
            rospy.logwarn("No '~relative_path' parameter found in parameter server, using default value '%s'", relative_path)
        if rospy.has_param("~model_name_in_simulation"):
            self.model_name = rospy.get_param("~model_name_in_simulation")
        else:
            self.model_name = "turtlebot2"
            rospy.logwarn("No '~model_name_in_simulation' parameter found in parameter server, using default value '%s'", model_name)
        if rospy.has_param("~footprint_side"):
            self.side_of_squared_footprint = rospy.get_param("~footprint_side")
        else:
            self.side_of_squared_footprint = 0.2
            rospy.logwarn("No '~footprint_side' parameter found in parameter server, using default value '%.2f'", self.side_of_squared_footprint)

        # check if 'static_map' service exists
        rospy.loginfo("Waiting for service 'static_map'...")
        static_map_service_found = False
        while not static_map_service_found:
          service_list = rosservice.get_service_list()
          if '/static_map' in service_list:
            static_map_service_found = True
          else:
            rospy.loginfo("Could not find 'static_map' service, is map_server running?")
            rospy.sleep(5)

        while not self.got_occupancy_grid:
          try:
            rospy.wait_for_service('static_map', timeout=5)
            mapService = rospy.ServiceProxy('static_map', GetMap)
            # parse OccupancyGrid object
            self.occupancy_grid = mapService().map.data
            self.map_width = mapService().map.info.width
            self.map_height = mapService().map.info.height
            self.map_resolution = mapService().map.info.resolution
            pixel_str = str(self.map_width)+"x"+str(self.map_height)
            meters_str = str(self.map_width*self.map_resolution)[:5]+"x"+ str(self.map_height*self.map_resolution)[:5]
            rospy.loginfo("Got map "+pixel_str+" pixel, or "+ meters_str+" meters")
            self.map_origin_x = mapService().map.info.origin.position.x
            self.map_origin_y = mapService().map.info.origin.position.y
            rospy.logdebug("Got map_origin_x, map_origin_y: "+ str(self.map_origin_x) +", "+str(self.map_origin_y))
            self.got_occupancy_grid = True
          except rospy.ServiceException, e:
            rospy.logerr("Service call to 'static_map' failed: {}".format(e))
            rospy.logwarn("Waiting to retry...")  
            rospy.sleep(5)

        rospy.loginfo("Got map from map_server!")
        self.generate_cspace_map(self.occupancy_grid)
        self.parse_empty_cells(self.cspace_map)


    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (self.publish_cmap):
                self.map_publisher.publish(self.map_msg)
            try:
                rate.sleep()
            except rospy.exceptions.ROSInterruptException as e:
                if rospy.core.is_shutdown():
                    break
                raise
if __name__ == '__main__':
    spawn_model = SpawnInFreeSpace()
    spawn_model.init()
    if spawn_model.detect_model():
      # find new random empty cell and move model
      spawn_model.move_model_random()
    else:
      # find random empty cell and spawn model
      spawn_model.spawn()
    # spawn_model.spin()