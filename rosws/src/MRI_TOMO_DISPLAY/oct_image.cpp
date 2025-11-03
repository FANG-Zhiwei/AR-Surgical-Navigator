#include "rclcpp/rclcpp.hpp"
#include "unity_robotics_demo_msgs/msg/image.hpp"
#include <memory>
#include <opencv2/opencv.hpp>

class ImagePublisherNode : public rclcpp::Node
{
public:
    ImagePublisherNode() : Node("oct_image_publisher_node")
    {
        publisher_ = this->create_publisher<unity_robotics_demo_msgs::msg::Image>("oct_image_topic", 10);

        // Directory path containing the image files
        std::string directory_path = "/home/jimmy/ros2_ws/src/MRI_TOMO_DISPLAY/resized";

        // List image files in the directory
        std::vector<cv::String> image_files;
        cv::glob(directory_path, image_files);

        // Publish each image file
        for (const auto &image_file : image_files)
        {
            cv::Mat image = cv::imread(image_file, cv::IMREAD_COLOR);

            // Create a sensor_msgs::Image message
            unity_robotics_demo_msgs::msg::Image image_msg;
            image_msg.header.stamp = this->now();
            image_msg.header.frame_id = "camera_frame";
            image_msg.height = image.rows;
            image_msg.width = image.cols;
            image_msg.encoding = "bgr8";
            image_msg.is_bigendian = 0;
            image_msg.data.resize(image.rows * image.cols * image.channels());

            memcpy(image_msg.data.data(), image.data, image.rows * image.cols * image.channels());

            // Publish the image message
            publisher_->publish(image_msg);
            RCLCPP_INFO(this->get_logger(), "Published image: %s", image_file.c_str());

            // Add a delay between publishing each image
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust the delay as needed
        }
    }

private:
    rclcpp::Publisher<unity_robotics_demo_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}