#!/usr/bin/env
import rospy
import tf

rospy.init_node('fake_tf')
rate = rospy.Rate(20)
tf_broadcaster = tf.TransformBroadcaster()
while not rospy.is_shutdown():
    tf_broadcaster.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), 'guidance', 'fake_tf')
    rate.sleep()
