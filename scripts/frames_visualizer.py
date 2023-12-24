#!/usr/bin/env /home/airsurf/miniconda3/envs/ROS/bin/python
# -*- coding: utf-8 -*-

import os

import numpy as np
import rospy
import cv2
import imgviz

from orb_slam3_ros_wrapper.msg import frame

class FramesVisualizer:
    
    def __init__(self) -> None:
        print("frames_visualizer Node: starting", os.getpid())
        print("Waiting for first frame...")

        rospy.init_node("frames_visualizer", anonymous=True)
        rospy.Subscriber("/frames_decompressed", frame, self.callback)
        rospy.spin()

    def callback(self, msg:frame):
        rgb_np = np.frombuffer(msg.rgb.data, dtype=np.uint8)
        rgb_np = rgb_np.reshape(msg.rgb.height, msg.rgb.width, 4)
        rgb_np = rgb_np[..., :3]
        rgb_np = rgb_np[..., ::-1]

        depth_np = np.frombuffer(msg.depth.data, dtype=np.float32)
        depth_np = depth_np.reshape(msg.depth.height, msg.depth.width)
        
        depth_viz = imgviz.depth2rgb(depth_np)[..., ::-1]
        
        viz = np.hstack((rgb_np, depth_viz))
        cv2.imshow('rgbd', viz)
        if cv2.waitKey(1) > 0:
            cv2.destroyAllWindows()
            rospy.signal_shutdown("User pressed any key")
        
        return
    
if __name__ == "__main__":
    FramesVisualizer()