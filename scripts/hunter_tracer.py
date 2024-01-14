#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import numpy as np

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
import tf_conversions

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
        if len(dis):
            idx = max(1, np.argmin(dis))
            self.index = self.index+idx
            rospy.loginfo("Tracing at index: {}, totally {}".format(self.index, self.ncourse))
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

class HunterTracer:
    
    __linear_velocity = 0.3
    __yaw_threshold = np.pi / 36
    
    def __init__(self):
        rospy.init_node('hunter_tracer')
        
        self.__current_pose = None
        self.__cancel_flag = False
        
        rospy.Subscriber('/hunter_path', Path, self.__path_callback)
        rospy.Subscriber('/localization', Odometry, self.__localization_callback)
        rospy.Subscriber('/hunter_tracer/cancel', Bool, self.__cancel_callback)
        self.__cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.spin()
        
    def __path_callback(self, msg:Path):
        self.__cancel_flag = False
        
        waypoints = []
        for pose_stamped in msg.poses:
            pose_stamped:PoseStamped
            pose = pose_stamped.pose
            waypoints.append((pose.position.x, pose.position.y))
        waypoints = np.array(waypoints)
        
        rpath = ReferencePath(waypoints)
        while not rospy.is_shutdown():
            if self.__cancel_flag:
                rospy.loginfo("Cancel flag is set")
                break
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
            current_twist.linear.x = self.__linear_velocity
            dis_x = xref[0] - current_x
            dis_y = xref[1] - current_y
            dis_yaw = np.arctan2(dis_y, dis_x) - current_yaw
            dis_yaw = np.arctan2(np.sin(dis_yaw), np.cos(dis_yaw))
            
            if -np.pi <= dis_yaw < -(np.pi - self.__yaw_threshold):
                current_twist.angular.z = 0
            elif -(np.pi - self.__yaw_threshold) <= dis_yaw < -np.pi * 0.5:
                current_twist.angular.z = -1
            elif -np.pi * 0.5 <= dis_yaw < -self.__yaw_threshold:
                current_twist.angular.z = -1
            elif -self.__yaw_threshold <= dis_yaw < self.__yaw_threshold:
                current_twist.angular.z = 0
            elif self.__yaw_threshold <= dis_yaw < np.pi * 0.5:
                current_twist.angular.z = 1
            elif np.pi * 0.5 <= dis_yaw < (np.pi - self.__yaw_threshold):
                current_twist.angular.z = 1
            elif (np.pi - self.__yaw_threshold) <= dis_yaw <= np.pi:
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
        
    def __cancel_callback(self, msg:Bool):
        self.__cancel_flag = msg.data
        
if __name__ == '__main__':
    hunter_tracer = HunterTracer()
    