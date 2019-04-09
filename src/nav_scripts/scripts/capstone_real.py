#! /usr/bin/env python
import roslib
roslib.load_manifest('nav_scripts')
import sys
import traceback
import time
import rospy
import actionlib
import roslaunch
import rospkg
import time
from nav_scripts.srv import *
from nav_scripts.msg import NavigatorAction, NavigatorGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion, PoseStamped

TAKEOFF_TASK = 1
LANDING_TASK = 2
ACTIVATION_TASK = 5

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

            self.landing_site_sub = rospy.Subscriber('/LSD/landing_site', PoseStamped, self.landing_cb)
            self.pose_sub = rospy.Subscriber('/guidance/pose', PoseStamped, self.pose_cb)
            self.landing_site = None
            self.curr_pose = None
            # ros pkg stuff
            self.rospack = rospkg.RosPack()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.basesrvFlag_=False
        except Exception as e_init:
            tracebackString = traceback.format_exc(e_init)
            print tracebackString
        rospy.loginfo('init finished')

    def landing_cb(self, pose):
        self.landing_site = pose.pose.position 

    def pose_cb(self, pose):
        self.curr_pose = pose.pose.position

    def register_ros(self):
        rospy.loginfo('register_ros')
        self.navigator_client.wait_for_server()
        if(self.basesrvFlag_==False):
            rospy.logdebug('blocking call for basetasks srv')
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
        rospy.loginfo(self.navigator_client.get_goal_status_text() + str(rospy.Time.now().to_sec() - a))

    def move_in_square(self, edge_size, z=3.0):
        self.navigator_goal(coord_flag='GNSS_BODY_ROBOT',x=edge_size,y=0.0,z=z,yaw=0.0)
        self.navigator_goal(coord_flag='GNSS_BODY_ROBOT',x=0.0,y=edge_size,z=z,yaw=0.0)
        self.navigator_goal(coord_flag='GNSS_BODY_ROBOT',x=-1 * edge_size,y=0.0,z=z,yaw=0.0)
        self.navigator_goal(coord_flag='GNSS_BODY_ROBOT',x=0.0,y=-1*edge_size,z=z,yaw=0.0)

    def go_home(self, z=3.0):
        self.navigator_goal(coord_flag='GNSS_BODY_HOME', x=0, y=0, z=z, yaw=0)

    def run_code(self):
        rospy.loginfo('Mission starting')

        activation  = self.baseSrv(task=ACTIVATION_TASK)
        if activation.result is False:
            rospy.logerr('Activation service call failed. Exiting.')
            return
        rospy.loginfo('Activation service call succeeded. Calling for takeoff.')

        rospy.sleep(2)

        takeoff = self.baseSrv(task=TAKEOFF_TASK)
        if takeoff.result is False:
            rospy.logerr('Takeoff service call failed. Exiting')
            return
        rospy.loginfo('Takeoff service call succeeded. Making a square.')

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # launch orb slam
        rospy.loginfo('Launching orb slam node')
        orb_slam_launch_path = self.rospack.get_path('orb_slam2_ros') + '/ros/launch/capstone.launch'
        rospy.loginfo(orb_slam_launch_path)
        orb_launch = roslaunch.parent.ROSLaunchParent(uuid, [orb_slam_launch_path])
        orb_launch.start()
        time.sleep(5)

        rospy.loginfo('Moving in square')
        self.move_in_square(3, z=3)
        self.go_home(z=3)

        rospy.loginfo('launching voxblox')
        voxblox_launch_path = self.rospack.get_path('voxblox_ros') + '/launch/capstone_sim.launch'
        rospy.loginfo(voxblox_launch_path)
        voxblox_launch = roslaunch.parent.ROSLaunchParent(uuid, [voxblox_launch_path])
        voxblox_launch.start()
        time.sleep(5)

        rospy.loginfo('Moving in square')
        self.move_in_square(3, z=3)

        rospy.logwarn('launching landing site detection')
        lsd_launch_path = self.rospack.get_path('landing_site_detection') + '/launch/capstone.launch'
        lsd_launch = roslaunch.parent.ROSLaunchParent(uuid, [lsd_launch_path])
        lsd_launch.start()
     
        rospy.loginfo('sleeping 20 sec')
        rospy.sleep(10.)

        if self.landing_site is not None:
            landing_site_x = self.landing_site.x - self.curr_pose.x
            landing_site_y = self.landing_site.y - self.curr_pose.y
            self.navigator_goal(coord_flag='GNSS_BODY_ROBOT', x=landing_site_x, y=landing_site_y, z=3.0, yaw=0.0)
        else:
            rospy.logerr("No landing site")

        rospy.loginfo('landing')
        land = self.baseSrv(task=LANDING_TASK)
        if land.result is False:
            print 'Landing service call failed. Exiting'
            return
        
if __name__ == '__main__':
    robot_ns_ = rospy.get_param('robot_ns','/uas4')
    print robot_ns_
    squares = Makesquares_east_left(robot_ns_)
    squares.register_ros()
    squares.run_code()
