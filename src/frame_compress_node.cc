#include <frame_transmission/common.h>

#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class frame_compress_node
{
private:
    ros::NodeHandle* node_handler;

    ros::Publisher frame_compressed_pub;

    tf2::Transform align_transform;

    bool is_aligned = false;

public:
    frame_compress_node(ros::NodeHandle* node_handler) 
    : node_handler(node_handler)
    {
        align_transform.setIdentity();
        frame_compressed_pub = node_handler->advertise<orb_slam3_ros_wrapper::frame_compressed>("/frames_compressed", 1);
    }

    void frame_align_callback(const geometry_msgs::QuaternionConstPtr& msg)
    {
        if (!is_aligned) {
            double roll, pitch, yaw;
            align_transform.getBasis().getRPY(roll, pitch, yaw);
            tf2::Vector3 origin = align_transform.getOrigin();
            ROS_INFO("frame align old: roll: %f, pitch: %f, yaw: %f, x: %f, y: %f, z: %f", roll, pitch, yaw, origin.getX(), origin.getY(), origin.getZ());
            tf2::Quaternion q;
            tf2::convert(*msg, q);
            align_transform.setRotation(q);
            // origin.setY(origin.getY() + 2);
            // align_transform.setOrigin(origin);
            align_transform = align_transform.inverse();
            align_transform.getBasis().getRPY(roll, pitch, yaw);
            origin = align_transform.getOrigin();
            ROS_INFO("frame align new: roll: %f, pitch: %f, yaw: %f, x: %f, y: %f, z: %f", roll, pitch, yaw, origin.getX(), origin.getY(), origin.getZ());
            is_aligned = true;
            ROS_INFO("frame aligned");
        }
        return;
    }

    void frame_compress_callback(const frame_msg_ptr& msg)
    {   
        if (!is_aligned) {
            return;
        }
        int rgb_jpeg_quality = 95;

        sensor_msgs::CompressedImage msgRGB_compressed;
        const sensor_msgs::ImageConstPtr msgRGB = boost::make_shared<sensor_msgs::Image>(msg->rgb);

        ROS_INFO("rgb size: %i", msgRGB->data.size());
        msgRGB_compressed.header = msgRGB->header;

        msgRGB_compressed.format = msgRGB->encoding;

        int rgb_bit_depth = sensor_msgs::image_encodings::bitDepth(msgRGB->encoding);

        std::vector<int> rgb_compression_params;

        rgb_compression_params.reserve(2);
        rgb_compression_params.emplace_back(cv::IMWRITE_JPEG_QUALITY);
        rgb_compression_params.emplace_back(rgb_jpeg_quality);
        msgRGB_compressed.format += "; jpeg compressed ";
        ROS_INFO("rgb_bit_depth: %i", rgb_bit_depth);
        if ((rgb_bit_depth == 8) || (rgb_bit_depth == 16)) {
            std::string rgb_target_format;
            if (sensor_msgs::image_encodings::isColor(msgRGB->encoding)) {
                if (msgRGB->step == msgRGB->width * 3) {
                    rgb_target_format = "rgb8";
                } else if (msgRGB->step == msgRGB->width * 4) {
                    rgb_target_format = "bgra8";
                } else {
                    ROS_ERROR("Unsupported step size: %i", msgRGB->step);
                    return;
                }
            } else {
                rgb_target_format = "mono8";
            }
            msgRGB_compressed.format += rgb_target_format;
            ROS_INFO("rgb_target_format: %s", rgb_target_format.c_str());

            try {
                ROS_INFO("try to convert");
                cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msgRGB, rgb_target_format);

                ROS_INFO("try to compress");
                if (cv::imencode(".jpg", cv_ptr->image, msgRGB_compressed.data, rgb_compression_params)) {
                    ROS_INFO("Successfull jpeg compression with %i quality.", rgb_jpeg_quality);
                } else {
                    ROS_ERROR("cv::imencode (jpeg) failed on input image");
                    return;
                }
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            } catch (cv::Exception& e) {
                ROS_ERROR("cv::Exception: %s", e.what());
                return;
            }
        } else {
            ROS_ERROR("Unsupported bit depth %i, must be 8 or 16", rgb_bit_depth);
            return;
        }
        ROS_INFO("rgb_compressed size: %i", msgRGB_compressed.data.size());

        // depth png compression
        int depth_png_level = 4;
        double depth_max = 10.0;
        double depth_quantization = 100.0;
        std::string depth_format = "png";

        sensor_msgs::CompressedImage msgD_compressed;
        sensor_msgs::ImagePtr msgD = boost::make_shared<sensor_msgs::Image>(msg->depth);

        msgD_compressed.header = msgD->header;
        msgD_compressed.format = msgD->encoding;

        int depth_bit_depth = sensor_msgs::image_encodings::bitDepth(msgD->encoding);
        int num_channels_depth = sensor_msgs::image_encodings::numChannels(msgD->encoding);

        ConfigHeader depth_config;
        depth_config.format = INV_DEPTH;

        std::vector<uint8_t> depth_data;

        std::vector<int> depth_compression_params;

        depth_compression_params.resize(2, 0);
        depth_compression_params[0] = cv::IMWRITE_PNG_COMPRESSION;
        depth_compression_params[1] = depth_png_level;
        msgD_compressed.format += "; compressedDepth " + depth_format;

        if ((depth_bit_depth == 32) && (num_channels_depth == 1)) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msgD);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            const cv::Mat& depth_img = cv_ptr->image;
            size_t rows = depth_img.rows;
            size_t cols = depth_img.cols;
            ROS_INFO("depth size: %i", depth_img.total());

            if ((rows > 0) && (cols > 0)) {
                cv::Mat inv_depth_img = cv::Mat(rows, cols, CV_16UC1);

                float depth_quant_a = depth_quantization * (depth_quantization + 1.0f);
                float depth_quant_b = 1.0f - depth_quant_a / depth_max;

                cv::MatConstIterator_<float> it_depth_img = depth_img.begin<float>(), it_depth_img_end = depth_img.end<float>();
                cv::MatIterator_<unsigned short> it_inv_depth_img = inv_depth_img.begin<unsigned short>(), it_inv_depth_img_end = inv_depth_img.end<unsigned short>();

                for (; (it_depth_img != it_depth_img_end) && (it_inv_depth_img != it_inv_depth_img_end); ++it_depth_img, ++it_inv_depth_img) {
                    if (*it_depth_img <= depth_max) {
                        *it_inv_depth_img = depth_quant_a / *it_depth_img + depth_quant_b;
                    } else {
                        *it_inv_depth_img = 0;
                    }
                }

                depth_config.depthParam[0] = depth_quant_a;
                depth_config.depthParam[1] = depth_quant_b;

                if (depth_format == "png") {
                    try {
                        if (cv::imencode(".png", inv_depth_img, depth_data, depth_compression_params)) {
                            float depth_c_ratio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize()) / (float)depth_data.size();
                            ROS_INFO("Successfull png compression with level %i, compression ratio %f", depth_png_level, depth_c_ratio);
                        } else {
                            ROS_ERROR("cv::imencode (png) failed on input image");
                            return;
                        }
                    } catch (cv::Exception& e) {
                        ROS_ERROR("cv::Exception: %s", e.what());
                        return;
                    }
                } else {
                    ROS_ERROR("Unsupported depth format %s, must be png", depth_format.c_str());
                    return;
                }
            } else {
                ROS_ERROR("Empty depth image");
                return;
            }
        } else if ((depth_bit_depth == 16) && (num_channels_depth == 1)) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msgD);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            const cv::Mat& depth_img = cv_ptr->image;
            size_t rows = depth_img.rows;
            size_t cols = depth_img.cols;
            ROS_INFO("depth size: %i", depth_img.total());

            if ((rows > 0) && (cols > 0)) {
                unsigned short depth_max_ushort = static_cast<unsigned short>(depth_max * 1000.0f);

                cv::MatIterator_<unsigned short> it_depth_img = cv_ptr->image.begin<unsigned short>(), it_depth_img_end = cv_ptr->image.end<unsigned short>();

                for (; it_depth_img != it_depth_img_end; ++it_depth_img) {
                    if (*it_depth_img > depth_max_ushort) {
                        *it_depth_img = 0;
                    }
                }

                if (depth_format == "png") {
                    try {
                        if (cv::imencode(".png", depth_img, depth_data, depth_compression_params)) {
                            float depth_c_ratio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize()) / (float)depth_data.size();
                            ROS_INFO("Successfull png compression with level %i, compression ratio %f", depth_png_level, depth_c_ratio);
                        } else {
                            ROS_ERROR("cv::imencode (png) failed on input image");
                            return;
                        }
                    } catch (cv::Exception& e) {
                        ROS_ERROR("cv::Exception: %s", e.what());
                        return;
                    }
                } else {
                    ROS_ERROR("Unsupported depth format %s, must be png", depth_format.c_str());
                    return;
                }
            }
        }
        ROS_INFO("depth_compressed size: %i", depth_data.size());

        if (depth_data.size() > 0) {
            msgD_compressed.data.resize(sizeof(ConfigHeader));
            memcpy(&msgD_compressed.data[0], &depth_config, sizeof(ConfigHeader));
            msgD_compressed.data.insert(msgD_compressed.data.end(), depth_data.begin(), depth_data.end());
        } else {
            ROS_ERROR("Empty depth image");
            return;
        }

        geometry_msgs::Pose aligned_pose;
        tf2::Transform pose_transform;
        tf2::convert(msg->pose, pose_transform);
        double roll, pitch, yaw;
        pose_transform.getBasis().getRPY(roll, pitch, yaw);
        tf2::Vector3 origin = pose_transform.getOrigin();
        ROS_INFO("frame pose old: roll: %f, pitch: %f, yaw: %f, x: %f, y: %f, z: %f", roll, pitch, yaw, origin.getX(), origin.getY(), origin.getZ());
        pose_transform = align_transform * pose_transform;
        pose_transform.getBasis().getRPY(roll, pitch, yaw);
        origin = pose_transform.getOrigin();
        ROS_INFO("frame pose new: roll: %f, pitch: %f, yaw: %f, x: %f, y: %f, z: %f", roll, pitch, yaw, origin.getX(), origin.getY(), origin.getZ());
        tf2::toMsg(pose_transform, aligned_pose);
        

        orb_slam3_ros_wrapper::frame_compressed frame_msg_compressed;
        frame_msg_compressed.rgb = msgRGB_compressed;
        frame_msg_compressed.depth = msgD_compressed;
        frame_msg_compressed.pose = aligned_pose;
        // frame_msg_compressed.pose = msg->pose;

        frame_compressed_pub.publish(frame_msg_compressed);

        return;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frame_compress_node");
    ROS_INFO("frame_compress_node started");
    ros::NodeHandle node_handler;

    frame_compress_node frame_compress_node(&node_handler);

    ros::Subscriber frame_sub = node_handler.subscribe("/frames", 1, &frame_compress_node::frame_compress_callback, &frame_compress_node);
    ros::Subscriber align_msg_sub = node_handler.subscribe("/ground_rotation_quaternion", 1, &frame_compress_node::frame_align_callback, &frame_compress_node);

    ros::spin();

    return 0;
}