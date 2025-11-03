#include <memory>
#include "MRISliceExtractor.h"
#include "rclcpp/rclcpp.hpp"
#include "unity_robotics_demo_msgs/msg/needle_pos.hpp" // Include the message type header
#include "unity_robotics_demo_msgs/msg/image.hpp" 

using std::placeholders::_1;

class mri_image_process : public rclcpp::Node
{
public:
    mri_image_process()
        : Node("mri_image_process")
    {
        publisher_ = this->create_publisher<unity_robotics_demo_msgs::msg::Image>("image_output_topic", 10);
        subscription_ = this->create_subscription<unity_robotics_demo_msgs::msg::NeedlePos>(
            "needle_pos", 10, std::bind(&mri_image_process::needlePosCallback, this, _1));
    }

private:
    void needlePosCallback(const unity_robotics_demo_msgs::msg::NeedlePos::SharedPtr msg)
    {
        // Process the received needle position data here
        MRISlicer slicer("/home/jimmy/ros2_ws/src/MRI_TOMO_DISPLAY/IXI002-Guys-0828-T1.nii.gz");

        // Access the data fields in the message using msg->field_name
        double x = msg->needle_x;
        double y = msg->needle_y;
        double z = msg->needle_z;
        double rot_x = msg->rot_x;
        double rot_y = msg->rot_y;
        double rot_z = msg->rot_z;

        // main loop
        // recieve msgs from unity publisher node
        int x_pos = x;   // 0-200 -> -100 to 100
        int y_pos = y;   // 0-200 -> -100 to 100
        int z_pos = z;   // 0-200 -> -100 to 100
        int yaw_pos = rot_x;   // 0-360 -> -180 to 180
        int pitch_pos = rot_y; // 0-360 -> -180 to 180
        int roll_pos = rot_z;  // 0-360 -> -180 to 180

        // set up slice image parameters
        MRISlicer::SliceParameters params;
        params.x = x_pos;
        params.y = y_pos;
        params.z = z_pos;
        params.yaw = yaw_pos;
        params.pitch = pitch_pos;
        params.roll = roll_pos;
        // params.width_grid = params.height_grid = 240;
        // params.scale = 5.0;

        slicer.setSliceParameters(params);

        // get slice image
        cv::Mat sliceImage = slicer.getSlice();
        std::string filename = "/home/jimmy/ros2_ws/src/MRI_TOMO_DISPLAY/received_image.jpg";
        cv::imwrite(filename, sliceImage);

        // Convert the OpenCV Mat image to a sensor_msgs::Image message
        unity_robotics_demo_msgs::msg::Image image_msg;
        image_msg.header.stamp = this->now();
        image_msg.header.frame_id = "camera_frame";
        image_msg.height = sliceImage.rows;
        image_msg.width = sliceImage.cols;
        image_msg.encoding = "mono8";
        image_msg.is_bigendian = 0;
        image_msg.data.resize(sliceImage.rows * sliceImage.cols); // Assuming 3 channels (BGR)

        // Copy the image data from OpenCV Mat to the message
        memcpy(image_msg.data.data(), sliceImage.data, sliceImage.rows * sliceImage.cols);

        
        // Publish the received image message
        publisher_->publish(image_msg);

        if (!sliceImage.empty())
        {
            // node->publishImage(sliceImage);
        }
        else
        {
            std::cerr << "Error loading image!" << std::endl;
            // return EXIT_FAILURE;
        }
    }
    rclcpp::Subscription<unity_robotics_demo_msgs::msg::NeedlePos>::SharedPtr subscription_;
    rclcpp::Publisher<unity_robotics_demo_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mri_image_process>());
    rclcpp::shutdown();
    return 0;

    return 0;
}