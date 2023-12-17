#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Pose.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "orb_slam3_ros_wrapper/frame.h"
#include "orb_slam3_ros_wrapper/frame_compressed.h"

typedef boost::shared_ptr<const orb_slam3_ros_wrapper::frame_compressed> frame_compressed_msg_ptr;

class frame_decompress_node
{
private:
    ros::NodeHandle* node_handler;

    ros::Publisher frame_decompressed_pub;

public:
    frame_decompress_node(ros::NodeHandle* node_handler) 
    : node_handler(node_handler)
    {
        frame_decompressed_pub = node_handler->advertise<orb_slam3_ros_wrapper::frame>("/frames_decompressed", 1);
    }

    void frame_decompress_callback(const frame_compressed_msg_ptr& msg)
    {
        
    }

};