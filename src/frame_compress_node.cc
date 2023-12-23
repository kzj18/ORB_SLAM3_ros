#include <frame_transmission/common.h>

class frame_compress_node
{
private:
    ros::NodeHandle* node_handler;

    ros::Publisher frame_compressed_pub;

public:
    frame_compress_node(ros::NodeHandle* node_handler) 
    : node_handler(node_handler)
    {
        frame_compressed_pub = node_handler->advertise<orb_slam3_ros_wrapper::frame_compressed>("/frames_compressed", 1);
    }

    void frame_compress_callback(const frame_msg_ptr& msg)
    {
        int rgb_jpeg_quality = 95;

        sensor_msgs::CompressedImage msgRGB_compressed;
        sensor_msgs::ImagePtr msgRGB = boost::make_shared<sensor_msgs::Image>(msg->rgb);


        msgRGB_compressed.header = msgRGB->header;

        msgRGB_compressed.format = msgRGB->encoding;

        int rgb_bit_depth = sensor_msgs::image_encodings::bitDepth(msgRGB->encoding);

        std::vector<int> rgb_compression_params;

        rgb_compression_params.reserve(2);
        rgb_compression_params.emplace_back(cv::IMWRITE_JPEG_QUALITY);
        rgb_compression_params.emplace_back(rgb_jpeg_quality);
        msgRGB_compressed.format += "; jpeg compressed ";
        if ((rgb_bit_depth == 8) || (rgb_bit_depth == 16)) {
            std::string rgb_target_format;
            if (sensor_msgs::image_encodings::isColor(msgRGB->encoding)) {
                rgb_target_format = "bgr8";
            } else {
                rgb_target_format = "mono8";
            }
            msgRGB_compressed.format += rgb_target_format;

            try {
                cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msgRGB, rgb_target_format);

                if (cv::imencode(".jpg", cv_ptr->image, msgRGB_compressed.data, rgb_compression_params)) {
                    ROS_DEBUG("Successfull jpeg compression with %i quality.", rgb_jpeg_quality);
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

        depth_compression_params.resize(3, 0);
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
                            ROS_DEBUG("Successfull png compression with level %i, compression ratio %f", depth_png_level, depth_c_ratio);
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
        }

        if (depth_data.size() > 0) {
            msgD_compressed.data.resize(sizeof(ConfigHeader));
            memcpy(&msgD_compressed.data[0], &depth_config, sizeof(ConfigHeader));
            msgD_compressed.data.insert(msgD_compressed.data.end(), depth_data.begin(), depth_data.end());
        } else {
            ROS_ERROR("Empty depth image");
            return;
        }

        orb_slam3_ros_wrapper::frame_compressed frame_msg_compressed;
        frame_msg_compressed.rgb = msgRGB_compressed;
        frame_msg_compressed.depth = msgD_compressed;
        frame_msg_compressed.pose = msg->pose;

        frame_compressed_pub.publish(frame_msg_compressed);

        return;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frame_compress_node");
    ros::NodeHandle node_handler;

    frame_compress_node frame_compress_node(&node_handler);

    ros::Subscriber frame_sub = node_handler.subscribe("/frames", 1, &frame_compress_node::frame_compress_callback, &frame_compress_node);

    ros::spin();

    return 0;
}