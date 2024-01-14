#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import argparse

import cv2

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
import tf_conversions

from search import Search

class HunterPlanner:
    
    def __init__(self, config_file_path:str):
        rospy.init_node('hunter_planner')
        
        self.__search = Search(config_file_path)
        self.__start_point = None
        self.__end_point = None
        self.__waypoints = None
        self.__current_pose = None
        self.__cancel_enable = Bool()
        self.__cancel_enable.data = True
        self.__cancel_disable = Bool()
        self.__cancel_disable.data = False

        self.__map = cv2.cvtColor(self.__search.gridmap.showmap, cv2.COLOR_GRAY2BGR)
        
        self.__path_pub = rospy.Publisher('/hunter_path', Path, queue_size=1)
        self.__hunter_tracer_cancel_pub = rospy.Publisher('/hunter_tracer/cancel', Bool, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__goal_callback)
        rospy.Subscriber('/localization', Odometry, self.__localization_callback)
        self.__redraw_map_cv2()
        while not rospy.is_shutdown():
            key = cv2.waitKey(100) & 0xFF
            if key != 255:
                rospy.logdebug(key, repr(chr(key)))
            if key in [ord('q'), ord("\x1b")]:
                break
            self.__redraw_map_cv2()
                
    def __redraw_map_cv2(self):
        gridmap = self.__map.copy()
        if self.__waypoints is not None:
            for x_world, y_world in self.__waypoints:
                y_pixel, x_pixel = self.__search.gridmap.position2pixel(x_world, y_world)
                cv2.circle(gridmap, (x_pixel, y_pixel), 1, (0, 0, 255), cv2.FILLED)
        if self.__start_point is not None:
            y_pixel, x_pixel = self.__search.gridmap.position2pixel(*self.__start_point)
            cv2.circle(gridmap, (x_pixel, y_pixel), 1, (0, 255, 0), cv2.FILLED)
        if self.__end_point is not None:
            y_pixel, x_pixel = self.__search.gridmap.position2pixel(*self.__end_point)
            cv2.circle(gridmap, (x_pixel, y_pixel), 1, (0, 0, 255), cv2.FILLED)
        cv2.imshow('map', gridmap)
        
    def __goal_callback(self, msg:PoseStamped):
        if self.__current_pose is None:
            rospy.logwarn("Cannot get current pose")
            return
        self.__start_point = (self.__current_pose[0], self.__current_pose[1])
        self.__end_point = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo("Start point: {}, End point: {}".format(self.__start_point, self.__end_point))
        self.__waypoints = self.__search.astar_search(*self.__start_point, *self.__end_point)
        if self.__waypoints is None:
            rospy.logwarn("Cannot find path")
            return
        rospy.loginfo("Waypoints number: {}".format(len(self.__waypoints)))
        if self.__waypoints.shape[1] == 2:
            path = Path()
            path.header.frame_id = 'map'
            for current_x, current_y in self.__waypoints:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = current_x
                pose.pose.position.y = current_y
                path.poses.append(pose)
            for cancel_msg in [self.__cancel_enable, self.__cancel_disable]:
                for _ in range(3):
                    self.__hunter_tracer_cancel_pub.publish(cancel_msg)
                    rospy.sleep(0.05)
                
            self.__path_pub.publish(path)
    
    def __localization_callback(self, msg:Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf_conversions.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.__current_pose = (x, y, yaw)
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Hunter Controller.")
    parser.add_argument("--config", type=str, help="input yaml config", default="/home/airsurf/ws_lio/src/NEXTE_Sentry_Nav/sentry_slam/FAST_LIO/PCD/scans.yaml")
    
    args = parser.parse_args()
    hunter_planner = HunterPlanner(args.config)
    