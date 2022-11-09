#!/usr/bin/env python

"""
  @author Roberto Zegers
  @brief  Reinitializes AMCL particles to a new estimated mean and covariance
  @date   Aug 27, 2020
"""

import rospy
import sys
import tf.transformations as tft
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion

class InitPose():
  def __init__(self):
    self.initial_xyz             = [0,0,0]
    self.initial_rpy             = [0,0,0]
    self.initial_cov_xx          = 0.25
    self.initial_cov_yy          = 0.25
    # default: (PI/12)*(PI/12) rad,  PI*PI rad = 9.8696
    self.initial_cov_aa          = 0.068

  def usage():
    print '''Commands:
    -x <x in meters> - optional: initial pose, use 0 if left out
    -y <y in meters> - optional: initial pose, use 0 if left out
    -z <z in meters> - optional: initial pose, use 0 if left out
    -R <roll in radians> - optional: initial pose, use 0 if left out
    -P <pitch in radians> - optional: initial pose, use 0 if left out
    -Y <yaw in radians> - optional: initial pose, use 0 if left out
    -cov_xx <variance in m*m> - optional: initial pose covariance (x*x)
    -cov_yy <variance in m*m> - optional: initial pose covariance (y*y)
    -cov_aa <variance in rad*rad> - optional: initial pose covariance (yaw*yaw)
    '''

  def parse_user_inputs(self):
    # get coordinates from commandline
    for i in range(0,len(sys.argv)):
      if sys.argv[i] == '-h' or sys.argv[i] == '--help' or sys.argv[i] == '-help':
        usage()
        sys.exit(1)
      if sys.argv[i] == '-x':
        if len(sys.argv) > i+1:
          self.initial_xyz[0] = float(sys.argv[i+1])
      if sys.argv[i] == '-y':
        if len(sys.argv) > i+1:
          self.initial_xyz[1] = float(sys.argv[i+1])
      if sys.argv[i] == '-z':
        if len(sys.argv) > i+1:
          self.initial_xyz[2] = float(sys.argv[i+1])
      if sys.argv[i] == '-R':
        if len(sys.argv) > i+1:
          self.initial_rpy[0] = float(sys.argv[i+1])
      if sys.argv[i] == '-P':
        if len(sys.argv) > i+1:
          self.initial_rpy[1] = float(sys.argv[i+1])
      if sys.argv[i] == '-Y':
        if len(sys.argv) > i+1:
          self.initial_rpy[2] = float(sys.argv[i+1])
      if sys.argv[i] == '-cov_xx':
        if len(sys.argv) > i+1:
          self.initial_cov_xx = float(sys.argv[i+1])
      if sys.argv[i] == '-cov_yy':
        if len(sys.argv) > i+1:
          self.initial_cov_yy = float(sys.argv[i+1])
      if sys.argv[i] == '-cov_aa':
        if len(sys.argv) > i+1:
          self.initial_cov_aa = float(sys.argv[i+1])         

  def initialize_pose(self):
    rospy.init_node('amcl_init_pose')
    pub_pose = rospy.Publisher('/initialpose',PoseWithCovarianceStamped, latch=True, queue_size=10)
    rospy.sleep(1.0)

    initialpose_msg = PoseWithCovarianceStamped()
    initialpose_msg.header.seq = 0
    initialpose_msg.header.stamp.secs = 0
    initialpose_msg.header.stamp.nsecs = 0
    initialpose_msg.header.frame_id = "map"
    initialpose_msg.pose.pose.position.x = self.initial_xyz[0]
    initialpose_msg.pose.pose.position.y = self.initial_xyz[1]
    initialpose_msg.pose.pose.position.z = self.initial_xyz[2]
    # convert rpy to quaternion for Pose message
    tmpq = tft.quaternion_from_euler(self.initial_rpy[0],self.initial_rpy[1],self.initial_rpy[2])
    q = Quaternion(tmpq[0],tmpq[1],tmpq[2],tmpq[3])
    initialpose_msg.pose.pose.orientation = q
    initialpose_msg.pose.covariance[0]= self.initial_cov_xx
    initialpose_msg.pose.covariance[7]= self.initial_cov_yy
    initialpose_msg.pose.covariance[35]= self.initial_cov_aa

    pub_pose.publish(initialpose_msg)
    rospy.loginfo("Done reinitializing particles!")

if __name__ == "__main__":
    print("Started script to reinitialize AMCL particles")
    init_pose = InitPose()
    init_pose.parse_user_inputs()
    init_pose.initialize_pose()
