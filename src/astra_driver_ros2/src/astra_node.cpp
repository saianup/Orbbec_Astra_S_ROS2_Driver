#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <OpenNI.h>

#include <thread>
#include <atomic>
#include <vector>
#include <cstring>

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
            } else {
                msg.encoding = "rgb8";
                color_pub_->publish(msg);
            }
        }
    }

    // ROS publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;

    // OpenNI
    Device device_;
    VideoStream depth_stream_;
    VideoStream color_stream_;
    std::vector<VideoStream*> streams_;

    // Threading
    std::thread capture_thread_;
    std::atomic<bool> running_{true};
};


/// --------------------
/// REQUIRED main()
/// --------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AstraNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
