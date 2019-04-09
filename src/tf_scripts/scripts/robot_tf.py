#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

rospy.init_node('robot_pose')
tfl = tf.TransformListener()
pose_pub = rospy.Publisher('/guidance/robot_pose', PoseStamped, queue_size=1)
pose = PoseStamped()

def odom_cb(msg):
    global pose
    t = tfl.getLatestCommonTime('utm', 'uas4/gps')
    pose.header = msg.header
    pose.header.stamp = t
    pose.pose = msg.pose.pose

    pose = tfl.transformPose('uas4/gps', pose)
    pose.pose.orientation = msg.pose.pose.orientation
    pose.header.frame_id = 'world'
    pose_pub.publish(pose)

odom_sub = rospy.Subscriber('/uas4/simplenav/odom', Odometry, odom_cb)

if __name__ == '__main__':
    print 'running odom to pose'
    rospy.spin()