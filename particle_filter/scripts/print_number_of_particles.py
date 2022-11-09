#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray

def callback(data):
  # current number of particles
  particles = len(data.poses)
  rospy.loginfo_throttle(1,"Number of particles: %d"%particles)

if __name__ == '__main__':
  rospy.init_node('amcl_particle', anonymous=True)
  rospy.Subscriber("particlecloud", PoseArray, callback)
  rospy.spin()