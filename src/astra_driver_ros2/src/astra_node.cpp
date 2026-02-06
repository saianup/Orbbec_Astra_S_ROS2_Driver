#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <OpenNI.h>

#include <thread>
#include <atomic>
#include <vector>
#include <cstring>
#include <cmath>

using namespace openni;

class AstraNode : public rclcpp::Node
{
public:
    AstraNode()
    : Node("astra_camera_node")
    {
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "camera/depth/image_raw", 10);

        color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "camera/color/image_raw", 10);

        depth_info_pub_ =
            this->create_publisher<sensor_msgs::msg::CameraInfo>(
                "camera/depth/camera_info", 10);

        color_info_pub_ =
            this->create_publisher<sensor_msgs::msg::CameraInfo>(
                "camera/color/camera_info", 10);

        init_camera();

        capture_thread_ = std::thread(&AstraNode::capture_loop, this);
    }

    ~AstraNode()
    {
        running_ = false;

        if (capture_thread_.joinable())
            capture_thread_.join();

        if (depth_stream_.isValid()) depth_stream_.stop();
        if (color_stream_.isValid()) color_stream_.stop();

        depth_stream_.destroy();
        color_stream_.destroy();

        device_.close();
        OpenNI::shutdown();
    }

private:
    void init_camera()
    {
        OpenNI::initialize();
        device_.open(ANY_DEVICE);

        depth_stream_.create(device_, SENSOR_DEPTH);
        color_stream_.create(device_, SENSOR_COLOR);

        depth_stream_.start();
        color_stream_.start();

        streams_.push_back(&depth_stream_);
        streams_.push_back(&color_stream_);

        color_stream_.getProperty(
            ONI_STREAM_PROPERTY_HORIZONTAL_FOV, &color_hfov_);
        color_stream_.getProperty(
            ONI_STREAM_PROPERTY_VERTICAL_FOV, &color_vfov_);

        depth_stream_.getProperty(
            ONI_STREAM_PROPERTY_HORIZONTAL_FOV, &depth_hfov_);
        depth_stream_.getProperty(
            ONI_STREAM_PROPERTY_VERTICAL_FOV, &depth_vfov_);

        //std::cout << "Color HFOV: " << color_hfov_ << std::endl;
        //std::cout << "Color VFOV: " << color_vfov_ << std::endl;
    }

    sensor_msgs::msg::CameraInfo make_camera_info(
        int width, int height, float hfov, float vfov)
    {
        sensor_msgs::msg::CameraInfo info;

        info.header.frame_id = "camera_link";
        info.width = width;
        info.height = height;

        double fx = width / (2.0 * tan(hfov / 2.0));
        double fy = height / (2.0 * tan(vfov / 2.0));
        double cx = width / 2.0;
        double cy = height / 2.0;

        info.k = {
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        };

        info.p = {
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        };

        info.distortion_model = "plumb_bob";
        info.d = {0,0,0,0,0};

        return info;
    }

    void capture_loop()
    {
        while (rclcpp::ok() && running_) {
            int ready_index = -1;

            if (OpenNI::waitForAnyStream(
                    streams_.data(),
                    streams_.size(),
                    &ready_index,
                    2000) != STATUS_OK)
            {
                continue;
            }

            VideoFrameRef frame;
            streams_[ready_index]->readFrame(&frame);

            if (!frame.isValid())
                continue;

            sensor_msgs::msg::Image msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "camera_link";
            msg.width = frame.getWidth();
            msg.height = frame.getHeight();
            msg.is_bigendian = false;
            msg.step = frame.getStrideInBytes();
            msg.data.resize(frame.getDataSize());
            std::memcpy(msg.data.data(), frame.getData(), frame.getDataSize());

            if (streams_[ready_index] == &depth_stream_) {
                msg.encoding = "16UC1";
                depth_pub_->publish(msg);

                auto info = make_camera_info(
                    msg.width, msg.height,
                    depth_hfov_, depth_vfov_);
                info.header.stamp = msg.header.stamp;
                depth_info_pub_->publish(info);
            } else {
                msg.encoding = "rgb8";
                color_pub_->publish(msg);

                auto info = make_camera_info(
                    msg.width, msg.height,
                    color_hfov_, color_vfov_);
                info.header.stamp = msg.header.stamp;
                color_info_pub_->publish(info);
            }
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;

    Device device_;
    VideoStream depth_stream_;
    VideoStream color_stream_;
    std::vector<VideoStream*> streams_;

    float color_hfov_, color_vfov_;
    float depth_hfov_, depth_vfov_;

    std::thread capture_thread_;
    std::atomic<bool> running_{true};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AstraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

