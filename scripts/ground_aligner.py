#!/usr/bin/env /home/airsurf/miniconda3/envs/ROS/bin/python
# -*- coding: utf-8 -*-

import os
import argparse

import yaml
import numpy as np
import open3d as o3d

import rospy
import tf
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion

from orb_slam3_ros_wrapper.msg import frame

class GroundAligner:
    
    def __init__(self, config_file_path:str) -> None:
        assert os.path.exists(config_file_path), "config file does not exist"
        rospy.init_node("ground_aligner", anonymous=True)
        rospy.loginfo(f"ground_aligner Node: starting {os.getpid()}")
        
        self.__is_aligned = False
        self.__rotation_quaternion = None
        self.__height = None
        self.__delta = 1e-3
        with open(config_file_path, "r") as f:
            config_dict = yaml.load(f, Loader=yaml.FullLoader)
            self.__fx = float(config_dict["Camera.fx"])
            self.__fy = float(config_dict["Camera.fy"])
            self.__cx = float(config_dict["Camera.cx"])
            self.__cy = float(config_dict["Camera.cy"])
            self.__depth_factor = float(config_dict["DepthMapFactor"])
        self.__useful_depth = 1.5 / self.__depth_factor
        
        rospy.Subscriber("/frames", frame, self.callback, queue_size=1)
        self.__rotation_matrix_pub = rospy.Publisher("/ground_rotation_quaternion", Quaternion, queue_size=10)
        self.__height_pub = rospy.Publisher("/ground_height", Float32, queue_size=10)
        
        while not rospy.is_shutdown():
            if self.__is_aligned:
                self.__rotation_matrix_pub.publish(self.__rotation_quaternion)
                self.__height_pub.publish(self.__height)
            rospy.sleep(0.1)
    
    def callback(self, msg:frame):
        if self.__is_aligned:
            return
        else:
            rospy.loginfo("ground_aligner Node: aligning")
            pose_np = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            assert np.linalg.norm(pose_np) < self.__delta, f"position is not at the origin, it is at {pose_np}"
            orientation_np = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            assert np.linalg.norm(orientation_np - np.array([0, 0, 0, 1])) < self.__delta, f"orientation is not at the identity, it is at {orientation_np}"
            
            rgb_np = np.frombuffer(msg.rgb.data, dtype=np.uint8)
            rgb_np = rgb_np.reshape(msg.rgb.height, msg.rgb.width, msg.rgb.step // msg.rgb.width)
            rgb_np = rgb_np[..., :3]
            rgb_np = rgb_np[..., ::-1]
            
            depth_type = np.float32 if msg.depth.encoding == "32FC1" else np.uint16
            depth_np:np.ndarray = np.float32(np.frombuffer(msg.depth.data, dtype=depth_type)) / self.__depth_factor
            depth_np = depth_np.reshape(msg.depth.height, msg.depth.width)
            rows, cols = depth_np.shape
            
            c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
            valid = ~np.isnan(depth_np)
            z = np.where(valid, depth_np, np.nan)
            x = np.where(valid, z * (c - self.__cx) / self.__fx, np.nan)
            y = np.where(valid, z * (r - self.__cy) / self.__fy, np.nan)
            print("original z:", z.shape)
            z = z[valid]
            x = x[valid]
            y = y[valid]
            print("valid z:", z.shape)
            channel_r = rgb_np[..., 0][valid]
            channel_g = rgb_np[..., 1][valid]
            channel_b = rgb_np[..., 2][valid]
            
            useful = np.where(np.square(x) + np.square(y) + np.square(z) > np.square(self.__useful_depth))
            z = z[useful]
            x = x[useful]
            y = y[useful]
            print("useful z:", z.shape)
            channel_r = channel_r[useful]
            channel_g = channel_g[useful]
            channel_b = channel_b[useful]
            
            pc = np.dstack((x, y, z))
            pc_color = np.dstack((channel_r, channel_g, channel_b))
            pc = pc.reshape(-1, 3)
            pc_color = pc_color.reshape(-1, 3)
            print(pc.shape, pc_color.shape)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pc)
            pcd.colors = o3d.utility.Vector3dVector(pc_color / 255)
            plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
            [a, b, c, d] = plane_model
            origin = [0, 0, 0]
            normal = np.array([a, b, c])
            normal_length = np.linalg.norm(normal)
            height = (a * origin[0] + b * origin[1] + c * origin[2] + d) / normal_length
            if height == 0:
                target_normal = [0, 0, 1]
            else:
                target_normal = [0, 0, np.sign(height)]
            self.__height = abs(height)
            x_foot = origin[0] - a * height / normal_length
            y_foot = origin[1] - b * height / normal_length
            z_foot = origin[2] - c * height / normal_length

            rospy.loginfo(f"a: {a}, b: {b}, c: {c}, d: {d}, height: {self.__height}, x_foot: {x_foot}, y_foot: {y_foot}, z_foot: {z_foot}")
            inlier_cloud = pcd.select_by_index(inliers)
            inlier_cloud.paint_uniform_color([1.0, 0, 0])
            outlier_cloud = pcd.select_by_index(inliers, invert=True)
            normal_line = o3d.geometry.LineSet()
            normal_line.points = o3d.utility.Vector3dVector([origin, [x_foot, y_foot, z_foot]])
            normal_line.lines = o3d.utility.Vector2iVector([[0, 1]])
            normal_line.colors = o3d.utility.Vector3dVector([[1, 0, 0], [1, 0, 0]])
            o3d.visualization.draw_geometries([outlier_cloud, inlier_cloud, normal_line])

            normal = normal / normal_length
            cross_result = np.cross(normal, target_normal)
            cross_result_normalize = np.linalg.norm(cross_result)
            dot_result = np.dot(normal, target_normal)
            v_matrix = np.array([
                [0, -cross_result[2], cross_result[1]],
                [cross_result[2], 0, -cross_result[0]],
                [-cross_result[1], cross_result[0], 0]
            ])
            rotation_matrix = np.eye(3) + v_matrix + np.matmul(v_matrix, v_matrix) * (1 - dot_result) / (cross_result_normalize ** 2)
            # yaw = np.pi / 2
            # roll = 0
            # pitch = 0
            # from scipy.spatial.transform import Rotation as R
            # rotation_matrix = R.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
            
            self.__rotation_quaternion = Quaternion()
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            rospy.loginfo(f"transform matrix:\n{transform_matrix}")
            rotation_quaternion = tf.transformations.quaternion_from_matrix(transform_matrix)
            self.__rotation_quaternion.x = rotation_quaternion[0]
            self.__rotation_quaternion.y = rotation_quaternion[1]
            self.__rotation_quaternion.z = rotation_quaternion[2]
            self.__rotation_quaternion.w = rotation_quaternion[3]
            rospy.loginfo(f"transform quaternion: {rotation_quaternion}")
            rospy.loginfo(f"transform matrix from quaternion:\n{tf.transformations.quaternion_matrix(rotation_quaternion)}")

            inlier_cloud.rotate(rotation_matrix, center=origin)
            outlier_cloud.rotate(rotation_matrix, center=origin)
            normal_line.rotate(rotation_matrix, center=origin)
            x_line = o3d.geometry.LineSet()
            x_line.points = o3d.utility.Vector3dVector(np.array([[0, 0, 0], [1, 0, 0]]))
            x_line.lines = o3d.utility.Vector2iVector(np.array([[0, 1]]))
            x_line.colors = o3d.utility.Vector3dVector(np.array([[1, 0, 0], [1, 0, 0]]))
            y_line = o3d.geometry.LineSet()
            y_line.points = o3d.utility.Vector3dVector(np.array([[0, 0, 0], [0, 1, 0]]))
            y_line.lines = o3d.utility.Vector2iVector(np.array([[0, 1]]))
            y_line.colors = o3d.utility.Vector3dVector(np.array([[0, 1, 0], [0, 1, 0]]))
            z_line = o3d.geometry.LineSet()
            z_line.points = o3d.utility.Vector3dVector(np.array([[0, 0, 0], [0, 0, 1]]))
            z_line.lines = o3d.utility.Vector2iVector(np.array([[0, 1]]))
            z_line.colors = o3d.utility.Vector3dVector(np.array([[0, 0, 1], [0, 0, 1]]))
            o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud, x_line, y_line, z_line])
            self.__is_aligned = True
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ground Aligner: rotate the map so that the ground is horizontal")
    parser.add_argument("--config", type=str, required=True, help="input yaml config")
    arg, _ = parser.parse_known_args()
    
    GroundAligner(arg.config)