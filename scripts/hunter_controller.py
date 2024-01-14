#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import os
import argparse

import cv2

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import tf_conversions

from search import Search

WORKSPACE = os.path.dirname(os.path.abspath(__file__))

import numpy as np

class ReferencePath:
    def __init__(self, path:np.ndarray, pre_point=15, window_size=20):
        self.ncourse = path.shape[0]
        self.refer_path = path
        self.index = 0
        self.pre_point = pre_point
        self.window_size = window_size

    def update_param(self, pre_point, window_size):
        self.pre_point = pre_point
        self.window_size = window_size

    def calc_track_error(self, x, y):
        d_x = self.refer_path[:, 0] - x
        d_y = self.refer_path[:, 1] - y
        dis = np.hypot(d_x, d_y)
        self.index = np.argmin(dis)
        idx_end = min(self.index+self.window_size, self.refer_path.shape[0]-1)
        d_x = self.refer_path[self.index:idx_end, 0] - x
        d_y = self.refer_path[self.index:idx_end, 1] - y
        dis = np.hypot(d_x, d_y)
        rospy.loginfo("dis: {}".format(str(dis)))
        if len(dis):
            idx = max(1, np.argmin(dis))
            rospy.loginfo("idx: {}".format(idx))
            self.index = self.index+idx
            rospy.loginfo("index: {}".format(self.index))
        return self.index

    def calc_ref_trajectory(self, robot_state):
        index = self.calc_track_error(robot_state[0], robot_state[1])

        xref = np.zeros(2)
        approx = False
        if (index + self.pre_point) < self.ncourse:
            xref[0] = self.refer_path[index + self.pre_point, 0]
            xref[1] = self.refer_path[index + self.pre_point, 1]
        else:
            xref[0] = self.refer_path[self.ncourse - 1, 0]
            xref[1] = self.refer_path[self.ncourse - 1, 1]
            approx = True

        return xref, approx

class HunterController:
    
    def __init__(self, config_file_path:str):
        rospy.init_node('hunter_controller')
        
        self.__search = Search(config_file_path)
        self.__start_point = None
        self.__end_point = None
        self.__waypoints = None
        self.__current_pose = None

        self.__map = cv2.cvtColor(self.__search.gridmap.showmap, cv2.COLOR_GRAY2BGR)
        
        self.__cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.__path_pub = rospy.Publisher('/path', Path, queue_size=1)
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
            self.__path_pub.publish(path)
            
        rpath = ReferencePath(self.__waypoints)
        yaw_threshold = np.pi / 12
        while not rospy.is_shutdown():
            current_pose = self.__current_pose
            if current_pose is None:
                rospy.logwarn("Cannot get current pose")
                continue
            xref, approx = rpath.calc_ref_trajectory(current_pose)
            if approx:
                rospy.loginfo("Approximate end point")
                break
            current_x, current_y, current_yaw = current_pose
            current_twist = Twist()
            current_twist.linear.x = 0.1
            dis_x = xref[0] - current_x
            dis_y = xref[1] - current_y
            dis_yaw = np.arctan2(dis_y, dis_x) - current_yaw
            dis_yaw = np.arctan2(np.sin(dis_yaw), np.cos(dis_yaw))
            
            if -np.pi <= dis_yaw < -(np.pi - yaw_threshold):
                current_twist.angular.z = 0
            elif -(np.pi - yaw_threshold) <= dis_yaw < -np.pi * 0.5:
                current_twist.angular.z = 1
            elif -np.pi * 0.5 <= dis_yaw < -yaw_threshold:
                current_twist.angular.z = -1
            elif -yaw_threshold <= dis_yaw < yaw_threshold:
                current_twist.angular.z = 0
            elif yaw_threshold <= dis_yaw < np.pi * 0.5:
                current_twist.angular.z = 1
            elif np.pi * 0.5 <= dis_yaw < (np.pi - yaw_threshold):
                current_twist.angular.z = -1
            elif (np.pi - yaw_threshold) <= dis_yaw <= np.pi:
                current_twist.angular.z = 0
                
            if -np.pi / 2 < dis_yaw <= np.pi / 2:
                current_twist.linear.x *= 1
            else:
                current_twist.linear.x *= -1
                
            self.__cmd_vel_pub.publish(current_twist)
            
    
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
    hunter_controller = HunterController(args.config)
    