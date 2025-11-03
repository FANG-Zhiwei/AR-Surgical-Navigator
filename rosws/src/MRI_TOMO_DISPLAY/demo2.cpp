// main.cpp

#include "MRISliceExtractor.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>

// class ImagePublisher : public rclcpp::Node {
// public:
//     ImagePublisher() : Node("image_publisher") {
//         publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
//     }

//     void publishImage(cv::Mat &image) {
//         sensor_msgs::msg::Image ros_image;
//         ros_image.header.stamp = this->now();
//         ros_image.header.frame_id = "camera_frame";
//         ros_image.height = image.rows;
//         ros_image.width = image.cols;
//         ros_image.encoding = "bgr8"; // Assuming OpenCV image is in BGR format
//         ros_image.is_bigendian = false;
//         ros_image.step = image.cols * image.channels();
//         size_t size = image.total() * image.elemSize();
//         ros_image.data.resize(size);
//         memcpy(ros_image.data.data(), image.data, size);

//         publisher_->publish(ros_image);
//     }

// private:
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
// };

int main(int argc, char* argv[])
{
    // rclcpp::init(argc, argv);
    // auto node = std:: make_shared<ImagePublisher>();

    // 创建 OpenCV 窗口
    const std::string windowName = "MRI Slice";
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

    // 回调函数不需要实际使用，因为我们将在主循环中处理
    auto on_trackbar = [](int, void*) {};

    // 创建滑动条
    cv::createTrackbar("X", windowName, nullptr, 200, on_trackbar);
    cv::createTrackbar("Y", windowName, nullptr, 200, on_trackbar);
    cv::createTrackbar("Z", windowName, nullptr, 200, on_trackbar);
    cv::createTrackbar("Yaw", windowName, nullptr, 360, on_trackbar);
    cv::createTrackbar("Pitch", windowName, nullptr, 360, on_trackbar);
    cv::createTrackbar("Roll", windowName, nullptr, 360, on_trackbar);

    // 设置滑动条的初始位置
    cv::setTrackbarPos("X", windowName, 100);    // 初始值为0
    cv::setTrackbarPos("Y", windowName, 100);    // 初始值为0
    cv::setTrackbarPos("Z", windowName, 100);    // 初始值为0
    cv::setTrackbarPos("Yaw", windowName, 180);  // 初始值为0
    cv::setTrackbarPos("Pitch", windowName, 180);// 初始值为0
    cv::setTrackbarPos("Roll", windowName, 180); // 初始值为0

    // 创建MRISlicer对象并加载MRI数据
    MRISlicer slicer("/home/jimmy/ros2_ws/src/MRI_TOMO_DISPLAY/IXI002-Guys-0828-T1.nii.gz");

    // 主循环
    while (true)
    {
        // 获取滑动条的位置
        int x_pos = cv::getTrackbarPos("X", windowName) - 100;    // 0-200 -> -100 to 100
        int y_pos = cv::getTrackbarPos("Y", windowName) - 100;    // 0-200 -> -100 to 100
        int z_pos = cv::getTrackbarPos("Z", windowName) - 100;    // 0-200 -> -100 to 100
        int yaw_pos = cv::getTrackbarPos("Yaw", windowName) - 180;   // 0-360 -> -180 to 180
        int pitch_pos = cv::getTrackbarPos("Pitch", windowName) - 180; // 0-360 -> -180 to 180
        int roll_pos = cv::getTrackbarPos("Roll", windowName) - 180; // 0-360 -> -180 to 180

        // 定义切片参数
        MRISlicer::SliceParameters params;
        params.x = x_pos;
        params.y = y_pos;
        params.z = z_pos; // 假设z上限是200，取中间值100进行反转
        params.yaw = yaw_pos;   // 绕Z轴旋转45度
        params.pitch = pitch_pos; // 绕Y轴旋转30度
        params.roll = roll_pos;  // 绕X轴旋转60度
        params.width_grid = params.height_grid = 240;
        params.scale = 5.0;

        // 设置切片参数
        slicer.setSliceParameters(params);

        // 获取切片图像
        cv::Mat sliceImage = slicer.getSlice();

        // if (!sliceImage.empty()) {
        //     node->publishImage(sliceImage);
        // } else {
        //     std::cerr << "Error loading image!" << std::endl;
        // }

        if (sliceImage.empty())
        {
            std::cerr << "Failed to extract slice." << std::endl;
            return EXIT_FAILURE;
        }

        // 显示切片
        cv::imshow(windowName, sliceImage);

        // 检测按键，按下 Esc 键退出
        char key = static_cast<char>(cv::waitKey(30));
        if (key == 27) // Esc key
        {
            break;
        }

        // rclcpp::spin(node);
        // rclcpp::shutdown();

    }

    return EXIT_SUCCESS;
}
