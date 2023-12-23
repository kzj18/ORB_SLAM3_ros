#include <frame_transmission/common.h>

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
        ROS_INFO("frame_decompress_callback");
        sensor_msgs::Image msgRGB_decompressed;
        sensor_msgs::Image msgD_decompressed;
        
        int cfg_imdecode_flag = cv::IMREAD_UNCHANGED;

        cv_bridge::CvImagePtr cv_ptrRGB(new cv_bridge::CvImage);
        cv_ptrRGB->header = msg->rgb.header;
        ROS_INFO("msg->rgb.data.size(): %i", msg->rgb.data.size());
        try {
            cv_ptrRGB->image = cv::imdecode(msg->rgb.data, cfg_imdecode_flag);

            const size_t split_pos_RGB = msg->rgb.format.find(';');
            if (split_pos_RGB == std::string::npos) {
                switch (cv_ptrRGB->image.channels())
                {
                case 1:
                    cv_ptrRGB->encoding = sensor_msgs::image_encodings::MONO8;
                    break;
                case 3:
                    cv_ptrRGB->encoding = sensor_msgs::image_encodings::BGR8;
                    break;
                default:
                    ROS_ERROR("Unsupported number of channels: %i", cv_ptrRGB->image.channels());
                    return;
                }
            } else {
                std::string RGB_encoding = msg->rgb.format.substr(0, split_pos_RGB);

                cv_ptrRGB->encoding = RGB_encoding;

                if (sensor_msgs::image_encodings::isColor(RGB_encoding)) {
                    std::string compressed_encoding = msg->rgb.format.substr(split_pos_RGB);
                    bool compressed_color = compressed_encoding.find("compressed bgr8") != std::string::npos;

                    if (compressed_color) {
                        if ((RGB_encoding == sensor_msgs::image_encodings::RGB8) || (RGB_encoding == sensor_msgs::image_encodings::RGB16)) {
                            cv::cvtColor(cv_ptrRGB->image, cv_ptrRGB->image, cv::COLOR_BGR2RGB);
                        }
                        if ((RGB_encoding == sensor_msgs::image_encodings::RGBA8) || (RGB_encoding == sensor_msgs::image_encodings::RGBA16)) {
                            cv::cvtColor(cv_ptrRGB->image, cv_ptrRGB->image, cv::COLOR_BGR2RGBA);
                        }
                        if ((RGB_encoding == sensor_msgs::image_encodings::BGRA8) || (RGB_encoding == sensor_msgs::image_encodings::BGRA16)) {
                            cv::cvtColor(cv_ptrRGB->image, cv_ptrRGB->image, cv::COLOR_BGR2BGRA);
                        }
                    } else {
                        if ((RGB_encoding == sensor_msgs::image_encodings::BGR8) || (RGB_encoding == sensor_msgs::image_encodings::BGR16)) {
                            cv::cvtColor(cv_ptrRGB->image, cv_ptrRGB->image, cv::COLOR_RGB2BGR);
                        }
                        if ((RGB_encoding == sensor_msgs::image_encodings::BGRA8) || (RGB_encoding == sensor_msgs::image_encodings::BGRA16)) {
                            cv::cvtColor(cv_ptrRGB->image, cv_ptrRGB->image, cv::COLOR_RGB2BGRA);
                        }
                        if ((RGB_encoding == sensor_msgs::image_encodings::RGBA8) || (RGB_encoding == sensor_msgs::image_encodings::RGBA16)) {
                            cv::cvtColor(cv_ptrRGB->image, cv_ptrRGB->image, cv::COLOR_RGB2RGBA);
                        }
                    }
                }
                if (msg->rgb.format.find("jpeg") != std::string::npos && sensor_msgs::image_encodings::bitDepth(RGB_encoding) == 16) {
                    cv_ptrRGB->image.convertTo(cv_ptrRGB->image, CV_16U, 256.0);
                }
            }
        } catch(const std::exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        size_t rows = cv_ptrRGB->image.rows;
        size_t cols = cv_ptrRGB->image.cols;
        ROS_INFO("rgb rows: %i, cols: %i", rows, cols);

        if ((rows > 0) && (cols > 0)) {
            msgRGB_decompressed = *(cv_ptrRGB->toImageMsg());
            ROS_INFO("msgRGB_decompressed.data.size(): %i", msgRGB_decompressed.data.size());
        } else {
            ROS_ERROR("Empty RGB image");
            return;
        }

        cv_bridge::CvImagePtr cv_ptrD(new cv_bridge::CvImage);
        cv_ptrD->header = msg->depth.header;

        const size_t split_pos_D = msg->depth.format.find(';');
        const std::string depth_encoding = msg->depth.format.substr(0, split_pos_D);
        std::string depth_format = "png";
        cv_ptrD->encoding = depth_encoding;

        ROS_INFO("msg->depth.data.size(): %i", msg->depth.data.size());
        if (msg->depth.data.size() > sizeof(ConfigHeader)) {
            ROS_INFO("msg->depth.data.size() > sizeof(ConfigHeader)");
            ConfigHeader compressionConfig;
            memcpy(&compressionConfig, &msg->depth.data[0], sizeof(compressionConfig));
            ROS_INFO("memcpy(&compressionConfig, &msg->depth.data[0], sizeof(compressionConfig))");

            const std::vector<uint8_t> depth_data(msg->depth.data.begin() + sizeof(compressionConfig), msg->depth.data.end());

            ROS_INFO("depth_data.size(): %i", depth_data.size());

            float depth_quant_a, depth_quant_b;

            depth_quant_a = compressionConfig.depthParam[0];
            depth_quant_b = compressionConfig.depthParam[1];
            ROS_INFO("depth_quant_a: %f, depth_quant_b: %f", depth_quant_a, depth_quant_b);

            if (sensor_msgs::image_encodings::bitDepth(depth_encoding) == 32) {
                ROS_INFO("sensor_msgs::image_encodings::bitDepth(depth_encoding) == 32");
                cv::Mat depth_decompressed;
                try {
                    depth_decompressed = cv::imdecode(depth_data, cv::IMREAD_UNCHANGED);
                    ROS_INFO("depth_decompressed.rows: %i, depth_decompressed.cols: %i", depth_decompressed.rows, depth_decompressed.cols);
                } catch(const std::exception& e) {
                    ROS_ERROR("cv::imdecode exception: %s", e.what());
                    return;
                }

                size_t rows = depth_decompressed.rows;
                size_t cols = depth_decompressed.cols;

                if ((rows > 0) && (cols > 0)) {
                    ROS_INFO("depth_decompressed.rows: %i, depth_decompressed.cols: %i", rows, cols);
                    cv_ptrD->image = cv::Mat(rows, cols, CV_32FC1);

                    cv::MatIterator_<float> it_depth_img = cv_ptrD->image.begin<float>(), it_depth_img_end = cv_ptrD->image.end<float>();
                    cv::MatConstIterator_<unsigned short> it_inv_depth_img = depth_decompressed.begin<unsigned short>(), it_inv_depth_img_end = depth_decompressed.end<unsigned short>();

                    for (; (it_depth_img != it_depth_img_end) && (it_inv_depth_img != it_inv_depth_img_end); ++it_depth_img, ++it_inv_depth_img) {
                        if (*it_inv_depth_img) {
                            *it_depth_img = depth_quant_a / ((float)*it_inv_depth_img - depth_quant_b);
                        } else {
                            *it_depth_img = std::numeric_limits<float>::quiet_NaN();
                        }
                    }

                    msgD_decompressed = *(cv_ptrD->toImageMsg());
                } else {
                    ROS_ERROR("Empty depth image with 32 bit depth");
                    return;
                }
            } else {
                ROS_INFO("sensor_msgs::image_encodings::bitDepth(depth_encoding) != 32");
                try {
                    cv_ptrD->image = cv::imdecode(depth_data, cv::IMREAD_UNCHANGED);
                } catch(const std::exception& e) {
                    ROS_ERROR("cv::imdecode exception: %s", e.what());
                    return;
                }

                size_t rows = cv_ptrD->image.rows;
                size_t cols = cv_ptrD->image.cols;
                ROS_INFO("depth rows: %i, cols: %i", rows, cols);

                if ((rows > 0) && (cols > 0)) {
                    msgD_decompressed = *(cv_ptrD->toImageMsg());
                    ROS_INFO("msgD_decompressed.data.size(): %i", msgD_decompressed.data.size());
                } else {
                    ROS_ERROR("Empty depth image with 16 bit depth");
                    return;
                }
            }
        } else {
            ROS_ERROR("Empty depth image");
            return;
        }

        orb_slam3_ros_wrapper::frame msg_decompressed;
        msg_decompressed.rgb = msgRGB_decompressed;
        msg_decompressed.depth = msgD_decompressed;
        msg_decompressed.pose = msg->pose;

        frame_decompressed_pub.publish(msg_decompressed);

        return;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frame_decompress_node");
    ros::NodeHandle node_handler;

    frame_decompress_node frame_decompress_node(&node_handler);

    ros::Subscriber frame_compressed_sub = node_handler.subscribe("/frames_compressed", 1, &frame_decompress_node::frame_decompress_callback, &frame_decompress_node);

    ros::spin();

    return 0;
}