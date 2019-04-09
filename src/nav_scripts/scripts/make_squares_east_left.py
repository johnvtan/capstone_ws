#! /usr/bin/env python
import roslib
roslib.load_manifest('nav_scripts')
import sys
import traceback
import time
import rospy
import actionlib
from nav_scripts.srv import *
from nav_scripts.msg import NavigatorAction, NavigatorGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
class Makesquares_east_left():

    def __init__(self, robot_ns_):
        try:
            self.robot_ns_ = robot_ns_
            rospy.init_node('make_square',anonymous=True)
            #flight_Status subcriber - neglect it this time
            #base_controller srv
            self.baseSrv = rospy.ServiceProxy(self.robot_ns_+'/simplenav/basetasks', Basetasks)
            self.basesrvFlag_=True
            #NAVIGATOR_client
            self.navigator_client = actionlib.SimpleActionClient(self.robot_ns_+'/navigator', NavigatorAction)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.basesrvFlag_=False
        except Exception as e_init:
            tracebackString = traceback.format_exc(e_init)
            print tracebackString
        rospy.loginfo('init finished')

    def register_ros(self):
        rospy.loginfo('register_ros')
        self.navigator_client.wait_for_server()
        if(self.basesrvFlag_==False):
            print 'blocking call for basetasks srv'
            rospy.wait_for_service(self.robot_ns_+'/simplenav/basetasks', Basetasks)
            self.baseSrv = rospy.ServiceProxy(self.robot_ns_+'/simplenav/basetasks', Basetasks)

    def navigator_goal(self,coord_flag,x,y,z,yaw):
        navGoal = NavigatorGoal()

        navGoal.goalPose.header.stamp = rospy.Time.now()
        a =rospy.Time.now().to_sec()
        navGoal.goalPose.header.frame_id = coord_flag
        navGoal.goalPose.pose.position.y =y #lat
        navGoal.goalPose.pose.position.x =x #long
        navGoal.goalPose.pose.position.z =z #Altitude rel to takeoff
        geo_Quat = Quaternion(*quaternion_from_euler(0.0, 0.0, yaw))
        navGoal.goalPose.pose.orientation = geo_Quat
        self.navigator_client.send_goal(navGoal)
        self.navigator_client.wait_for_result()
        print self.navigator_client.get_goal_status_text(), rospy.Time.now().to_sec() - a

    def run_code(self):
        #check for flight on ground/ air
        #neglecting this step
        # 1
        rospy.loginfo('run_code')
        activation  = self.baseSrv(task=5)
        rospy.sleep(2)
        takeoff  = self.baseSrv(task=1)
        count =0
        #2
        while(count<1):
            print count
            rospy.loginfo('calling  navigator pt1')
            self.navigator_goal(coord_flag='GNSS_BODY_ROBOT',x=2.0,y=0.0,z=3.0,yaw=0.0)
            rospy.loginfo('calling navigator  pt2')
            self.navigator_goal(coord_flag='GNSS_BODY_ROBOT',x=0.0,y=2.0,z=3.0,yaw=0.0)
            rospy.loginfo('calling navigator  pt3')
            self.navigator_goal(coord_flag='GNSS_BODY_ROBOT',x=-2.0,y=0.0,z=3.0,yaw=0.0)
            rospy.loginfo('calling navigator  pt4')
            self.navigator_goal(coord_flag='GNSS_BODY_ROBOT',x=0.0,y=-2.0,z=3.0,yaw=0.0)
            count = count+1
        landing  = self.baseSrv(task=2)
        
if __name__ == '__main__':
    robot_ns_ = rospy.get_param('robot_ns','/uas4')
    print robot_ns_
    squares = Makesquares_east_left(robot_ns_)
    squares.register_ros()
    squares.run_code()
