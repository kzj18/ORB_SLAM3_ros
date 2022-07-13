/**
 *
 * Adapted from ORB-SLAM3: Examples/ROS/src/ros_rgbd.cc
 *
 */

#include "orb_slam3_ros_wrapper/ORB_SLAM3_interface.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ORB_SLAM_RGBD");

  ros::NodeHandle node_handler;
  std::string node_name = ros::this_node::getName();

  std::string voc_file, settings_file;
  node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
  node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");
  bool show_vis;
  node_handler.param<bool>(node_name + "/show_vis", show_vis, false);

  if (voc_file == "file_not_set" || settings_file == "file_not_set")
  {
    ROS_ERROR("Please provide voc_file and settings_file in the launch file");
    ros::shutdown();
    return 1;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::RGBD, show_vis);

  ORB_SLAM3_interface orb_slam_interface(&SLAM, &node_handler);
//   setup_tf_orb_to_ros(ORB_SLAM3::System::RGBD);

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(node_handler, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(node_handler, "/camera/depth_registered/image_raw", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
  sync.registerCallback(boost::bind(&ORB_SLAM3_interface::rgbd_callback, &orb_slam_interface, _1, _2));

  ros::spin();

  // Stop all threads
  SLAM.Shutdown();

  ros::shutdown();

  return 0;
}
