// main.cpp

#include "MRISliceExtractor.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[])
{
    // 创建MRISlicer对象并加载MRI数据
    MRISlicer slicer("/home/jimmy/ros2_ws/src/MRI_TOMO_DISPLAY/IXI002-Guys-0828-T1.nii.gz");

    // main loop
    while (true)
    {
        // 获取滑动条的位置
        int x_pos = 150;    // 0-200 -> -100 to 100
        int y_pos = 100;    // 0-200 -> -100 to 100
        int z_pos = 100;    // 0-200 -> -100 to 100
        int yaw_pos = 180;   // 0-360 -> -180 to 180
        int pitch_pos = 180; // 0-360 -> -180 to 180
        int roll_pos = 180; // 0-360 -> -180 to 180

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
        std::string filename = "/home/jimmy/ros2_ws/src/MRI_TOMO_DISPLAY/received_image.jpg";
        cv::imwrite(filename, sliceImage);

        if (!sliceImage.empty()) {
            // node->publishImage(sliceImage);
        } else {
            std::cerr << "Error loading image!" << std::endl;
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}
