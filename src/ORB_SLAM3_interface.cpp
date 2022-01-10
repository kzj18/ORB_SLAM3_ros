#include "orb_slam3_ros_wrapper/ORB_SLAM3_interface.h"
#include "sophus/geometry.hpp"

ORB_SLAM3_interface::ORB_SLAM3_interface(ORB_SLAM3::System* pSLAM, ros::NodeHandle* node_handle)
  : mpSLAM(pSLAM), node_handle(node_handle)
{
  pose_pub = node_handle->advertise<geometry_msgs::PoseStamped>("/orb_slam3_ros/camera", 1);
  //  map_points_pub = node_handle->advertise<sensor_msgs::PointCloud2>("orb_slam3_ros/map_points", 1);
  map_frame_pub = node_handle->advertise<orb_slam3_ros_wrapper::map_frame>("/map_frames", 1);

  //  rendered_image_pub = image_transport.advertise("orb_slam3_ros/tracking_image", 1);

  std::string node_name = ros::this_node::getName();
  node_handle->param<std::string>(node_name + "/map_frame_id", map_frame_id, "map");
  node_handle->param<std::string>(node_name + "/pose_frame_id", pose_frame_id, "pose");
}

void ORB_SLAM3_interface::rgbd_callback(const sensor_msgs::ImageConstPtr& msgRGB,
                                        const sensor_msgs::ImageConstPtr& msgD)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try
  {
    cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try
  {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Main algorithm runs here
  Sophus::SE3f Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());

  publish_map_frame(Tcw, *msgRGB, *msgD, ORB_SLAM3::System::STEREO);
  //  publish_tracking_mappoints(mpSLAM->GetTrackedMapPoints(), cv_ptrRGB->header.stamp);
}

void ORB_SLAM3_interface::publish_map_frame(Sophus::SE3f Tcw, sensor_msgs::Image msgRGB, sensor_msgs::Image msgD,
                                            ORB_SLAM3::System::eSensor sensor_type)
{
  Eigen::Isometry3d camera_tf;
  camera_tf.matrix() = Tcw.matrix().cast<double>();

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = map_frame_id;

  pose_msg.pose.position.x = camera_tf.translation().x();
  pose_msg.pose.position.y = camera_tf.translation().y();
  pose_msg.pose.position.z = camera_tf.translation().z();

  Eigen::Quaterniond q(camera_tf.linear());
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  orb_slam3_ros_wrapper::map_frame map_frame_msg;
  map_frame_msg.rgb = msgRGB;
  map_frame_msg.depth = msgD;
  map_frame_msg.pose = pose_msg.pose;

  map_frame_pub.publish(map_frame_msg);

  // publish_tf_transform(tf_transform, current_frame_time);
  // publish_pose_stamped(tf_transform, current_frame_time);
}