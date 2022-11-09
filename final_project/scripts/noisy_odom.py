#!/usr/bin/env python

'''
Modified version of: noisy_odom.py, Team Leonard, University of Birmingham Intelligent Robotics 2018
'''

import rospy
import math
import random
# gauss() is an inbuilt method of the random module
from random import gauss
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_srvs.srv import Empty, EmptyResponse
import tf2_ros

# sl = standard deviation of the linear velocity Gaussian noise
# sa = standard deviation of the angular velocity Gaussian noise
sl, sa = 0.1, 0.5


def rotateQuaternion(q_orig, yaw):
    """
    Converts a basic rotation about the z-axis (in radians) into the
    Quaternion notation required by ROS transform and pose messages.

    :Args:
       | q_orig (geometry_msgs.msg.Quaternion): to be rotated
       | yaw (double): rotate by this amount in radians
    :Return:
       | (geometry_msgs.msg.Quaternion) q_orig rotated yaw about the z axis
     """
    # Create a temporary Quaternion to represent the change in heading
    q_headingChange = Quaternion()

    p = 0
    y = yaw / 2.0
    r = 0

    sinp = math.sin(p)
    siny = math.sin(y)
    sinr = math.sin(r)
    cosp = math.cos(p)
    cosy = math.cos(y)
    cosr = math.cos(r)

    q_headingChange.x = sinr * cosp * cosy - cosr * sinp * siny
    q_headingChange.y = cosr * sinp * cosy + sinr * cosp * siny
    q_headingChange.z = cosr * cosp * siny - sinr * sinp * cosy
    q_headingChange.w = cosr * cosp * cosy + sinr * sinp * siny

    # Multiply new (heading-only) quaternion by the existing (pitch and bank)
    # quaternion. Order is important! Original orientation is the second
    # argument rotation which will be applied to the quaternion is the first
    # argument.
    return multiply_quaternions(q_headingChange, q_orig)


def multiply_quaternions( qa, qb ):
    """
    Multiplies two quaternions to give the rotation of qb by qa.

    :Args:
       | qa (geometry_msgs.msg.Quaternion): rotation amount to apply to qb
       | qb (geometry_msgs.msg.Quaternion): to rotate by qa
    :Return:
       | (geometry_msgs.msg.Quaternion): qb rotated by qa.
    """
    combined = Quaternion()

    combined.w = (qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z)
    combined.x = (qa.x * qb.w + qa.w * qb.x + qa.y * qb.z - qa.z * qb.y)
    combined.y = (qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x)
    combined.z = (qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w)
    return combined


def getHeading(q):
    """
    Get the robot heading in radians from a Quaternion representation.

    :Args:
        | q (geometry_msgs.msg.Quaternion): a orientation about the z-axis
    :Return:
        | (double): Equivalent orientation about the z-axis in radians
    """
    yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                     q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
    return yaw

"""
def simple_gaussian(odom):
    "Applies simple gaussian noise to current position and odometry readings."
    sp, sr = 0.01, 0.008
    pos = odom.pose.pose.position
    odom.pose.pose.position = Point(gauss(pos.x, sp), gauss(pos.y, sp), gauss(pos.z, sp))
    rot = odom.pose.pose.orientation
    odom.pose.pose.orientation = Quaternion(gauss(rot.x, sr), gauss(rot.y, sr), gauss(rot.z, sr), gauss(rot.w, sr))
    return odom
"""

def add_noise(odom):
    '''
        using the linear and angular velocities extracted from each odometry
        message: add noise to these velocities and add them to the current
        fictional position to keep track independently of the positions reported
        by the odometry.
    '''
    global cl_odom

    # If cl_odom is not defined, then it must be the first callback
    if 'cl_odom' not in globals():
        cl_odom = odom
    else:
        # Get velocities
        lv = odom.twist.twist.linear
        av = odom.twist.twist.angular
        dt = (odom.header.stamp - cl_odom.header.stamp).nsecs * 1e-9

        # Add noise to velocities (sl, sa: standard deviations)
        lv = Vector3(gauss(lv.x, sl), gauss(lv.y, sl), lv.z)
        av = Vector3(av.x, av.y, gauss(av.z, av.z * sa))

        # Apply velocities to orientation of last location
        cl_ori = cl_odom.pose.pose.orientation
        odom.pose.pose.orientation = rotateQuaternion(cl_ori, av.z * dt)
        odom.twist.twist.angular = av
        yaw = getHeading(odom.pose.pose.orientation) % (2 * math.pi)

        # Apply velocities to position of last location
        cl_pos = cl_odom.pose.pose.position
        fwd, drift = lv.x * dt, lv.y * dt
        c = math.cos(yaw)
        s = math.sin(yaw)
        odom.pose.pose.position.x = cl_pos.x + c * fwd + s * drift
        odom.pose.pose.position.y = cl_pos.y + s * fwd + c * drift
        odom.twist.twist.linear = lv

        # Set cl_odom to odom
        cl_odom = odom

        # broadcast transform
        if publish_tf:
            broadcast_tf(odom, rospy.Time.now())

def broadcast_tf(odom_msg, pub_time):
    # broadcast tf for noisy_odom w.r.t odom
    noisy_odom_transform = tf2_ros.TransformStamped()
    noisy_odom_transform.header.stamp = pub_time
    noisy_odom_transform.header.frame_id = "wheel_odom"
    noisy_odom_transform.child_frame_id = "base_footprint"

    noisy_odom_transform.transform.translation = odom_msg.pose.pose.position
    noisy_odom_transform.transform.rotation = odom_msg.pose.pose.orientation

    tf_broadcaster.sendTransform(noisy_odom_transform)

def odometry_callback(odom):
    global pub
    rospy.logdebug("Got perfect odom: {} {}".format(odom.pose.pose.position.x, odom.pose.pose.position.y)) 
    # Add noise to the odometry
    add_noise(odom)
    # Republish
    pub.publish(odom)
    rospy.logdebug("Pub noisy odom: {} {}".format(odom.pose.pose.position.x, odom.pose.pose.position.y)) 

def shutdown_callback(_):
    global shutdown_flag
    response = EmptyResponse()
    # set shutdown flag
    shutdown_flag = True
    return response

def clean_shutdown():
    rospy.loginfo("Shutting down noisy odometry node...")

if __name__ == '__main__':
    global pub, shutdown_flag, tf_broadcaster, publish_tf
    shutdown_flag = False
    publish_tf = True
    rospy.init_node('noisy_odometry')

    pub = rospy.Publisher('wheel_odom', Odometry, queue_size=1)
    rospy.Subscriber('odom', Odometry, odometry_callback)
    shutdown_service = rospy.Service('/noisy_odom/shutdown', Empty, shutdown_callback)
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.TransformBroadcaster()


    rospy.loginfo("Started noisy odometry publisher node")
    # cleanup on shutdown
    rospy.on_shutdown(clean_shutdown)
    # equivalent to spin()
    while not rospy.core.is_shutdown() and not shutdown_flag:
      rospy.rostime.wallsleep(0.5)
    rospy.Timer(rospy.Duration(1), rospy.signal_shutdown('Shutting down'), oneshot=True)
