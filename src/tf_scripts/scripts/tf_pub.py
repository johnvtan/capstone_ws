#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped

rospy.init_node('pose_pub')
tfl = tf.TransformListener()
pose_pub = rospy.Publisher('/guidance/pose', PoseStamped, queue_size=1)
rate = rospy.Rate(60)
message = None
br = tf.TransformBroadcaster()

while not rospy.is_shutdown():
    try:
        t = tfl.getLatestCommonTime('map', 'guidance')
        pos, quat = tfl.lookupTransform( 'map', 'guidance', t)

        message = PoseStamped()

        message.header.stamp = t 
        message.header.frame_id = 'world'
        message.pose.position.x = pos[0]
        message.pose.position.y = pos[1]
        message.pose.position.z = pos[2]

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)

        quat = tf.transformations.quaternion_from_euler(-pitch, roll, yaw)
        message.pose.orientation.x = quat[0]
        message.pose.orientation.y = quat[1]
        message.pose.orientation.z = quat[2]
        message.pose.orientation.w = quat[3]

        #print('published new')
        br.sendTransform((0,0,0), (tf.transformations.quaternion_from_euler(0,0,-np.pi/2)), t, 'world', 'map')
        pose_pub.publish(message)
    except Exception as e:
        #print(e)
        if message is not None:
            pose_pub.publish(message)

    rate.sleep()
