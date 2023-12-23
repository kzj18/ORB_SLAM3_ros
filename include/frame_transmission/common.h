#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include "orb_slam3_ros_wrapper/frame.h"
#include "orb_slam3_ros_wrapper/frame_compressed.h"

// Compression formats
enum compressionFormat
{
  UNDEFINED = -1, INV_DEPTH
};

// Compression configuration
struct ConfigHeader
{
  // compression format
  compressionFormat format;
  // quantization parameters (used in depth image compression)
  float depthParam[2];
};

typedef boost::shared_ptr<orb_slam3_ros_wrapper::frame const> frame_msg_ptr;
typedef boost::shared_ptr<orb_slam3_ros_wrapper::frame_compressed const> frame_compressed_msg_ptr;
