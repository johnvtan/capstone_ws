#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

homepoint = None

def odom_cb(msg):
    global homepoint
    if homepoint is None:
        homepoint = PoseStamped()
        homepoint.pose.position = msg.pose.pose.position

    pose_msg = PoseStamped()
    pose_msg.pose.position.x = msg.pose.pose.position.x - homepoint.pose.position.x
    pose_msg.pose.position.y = msg.pose.pose.position.y - homepoint.pose.position.y
    pose_msg.pose.position.z = msg.pose.pose.position.z - homepoint.pose.position.z
    pose_msg.pose.orientation = msg.pose.pose.orientation
    pose_msg.header.stamp = msg.header.stamp
    pose_msg.header.frame_id = 'map' 
    pose_pub.publish(pose_msg)


rospy.init_node('odom_to_pose')
pose_pub = rospy.Publisher('/guidance/pose', PoseStamped, 1)
odom_sub = rospy.Subscriber('/uas4/simplenav/odom', Odometry, odom_cb)

if __name__ == '__main__':
    print 'running odom to pose'
    rospy.spin()
