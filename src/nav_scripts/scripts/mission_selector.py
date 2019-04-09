#! /usr/bin/env python
from __future__ import print_function
import sys, yaml

import actionlib, rospy
import roslib
roslib.load_manifest('simplenav')
#roslib.load_manifest('fixluxinveniettag')
roslib.load_manifest('april_track')
roslib.load_manifest('wirelesscharging_station')
from simplenav.srv import *
from simplenav.msg import NavigatorAction, NavigatorGoal
import april_track.msg
from wirelesscharging_station.msg import *

def waypoint_client(task):
    client = actionlib.SimpleActionClient('/uas4/navigator', NavigatorAction)
    navGoal = NavigatorGoal()
    client.wait_for_server()

    # set the waypoint
    navGoal.goalPose.header.stamp = rospy.Time.now()
    navGoal.goalPose.header.frame_id = task["frame"]
    navGoal.goalPose.pose.position.y =task["wp"][0] #lat
    navGoal.goalPose.pose.position.x =task["wp"][1]
    navGoal.goalPose.pose.position.z =task["wp"][2]
    navGoal.goalPose.pose.orientation.x =0
    navGoal.goalPose.pose.orientation.y =0
    navGoal.goalPose.pose.orientation.z =0
    navGoal.goalPose.pose.orientation.w =1

    client.send_goal(navGoal)
    print("start going to waypoint:")
    print(navGoal)

    client.wait_for_result()#rospy.Duration(task["timeout"]))
    print(client.get_result())
    return client.get_result()


def landing_client():
    client = actionlib.SimpleActionClient('/uas4/april_track_node/landing_module_act', april_track.msg.moduleAction)

    client.wait_for_server()
    goal = april_track.msg.moduleGoal(order=0)
    print('tag goal sent')

    client.send_goal(goal)
    client.wait_for_result()
    rospy.sleep(5)

    return client.get_result()

def align_client():
    try: 
        client = actionlib.SimpleActionClient('/uas4/wirelessCharger',WcmoduleAction)
        client.wait_for_server()
        goal = WcmoduleGoal()
        goal.EngageModule = True
        client.send_goal(goal)
        print("goal sent")
        client.wait_for_result()
        print("ack received")

        rospy.sleep(90)

        goal.EngageModule = False
        client.send_goal(goal)
        client.wait_for_result()
        print('unalign')
        rospy.sleep(20)
    except Exception as e:
        print(e)


if __name__ == '__main__':
    # readin mission file
    path = "/home/uas4/apps/robot_ws/src/uas_stack/simplenav/missions/debug_mission.yaml"
    config_file = open(path, 'r')
    config = yaml.safe_load(config_file)

    rospy.init_node('MissionPlanner')

    # setup home (emergency) position
    home_pt = config["home"][0]["wp"]
    missions = config["mission"]

    uav_srv_proxy = rospy.ServiceProxy('/uas4/simplenav/basetasks', Basetasks)

    num_of_tasks = len(missions)
    print("get", num_of_tasks, "missions, mission start!")

    i = 0
    while i < num_of_tasks:

        task = missions[i]

        if task["type"] == "takeoff":
            uav_srv_proxy(task=5) # arm the drone
            rospy.sleep(2)
            uav_srv_proxy(task=1) # takeoff
            rospy.sleep(5)
            print("successfully take off")

        elif task["type"] == "waypoint":
            result = waypoint_client(task)

        elif task["type"] == "land":
            if task["method"] == "apriltag":
                result = landing_client()
            
            if not result.landed:
                print("failed to land, retry")
                rospy.sleep(5)
                i -= 1
            else:
                print("landed!")

        elif task["type"] == "aligncharger":
            align_client()

        i += 1


