#include "realTimeDisplay.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

void RealTimeDisplay::display_image(k4a::image& k4a_image, std::string window_name)
{
    // 获取图像的宽度、高度和指针
    int width = k4a_image.get_width_pixels();
    int height = k4a_image.get_height_pixels();
    const uint8_t* buffer = k4a_image.get_buffer();

    // 检查图像格式并根据需要进行转换
    cv::Mat image;
    if (k4a_image.get_format() == K4A_IMAGE_FORMAT_COLOR_BGRA32) {
        // 如果是 BGRA32 格式（RGBA），将其转换为 OpenCV 的 BGRA 图像
        image = cv::Mat(height, width, CV_8UC4, (void*)buffer);
    }
    else if (k4a_image.get_format() == K4A_IMAGE_FORMAT_DEPTH16) {
        // 如果是深度图，直接使用 16 位无符号整数类型
        image = cv::Mat(height, width, CV_16U, (void*)buffer);
        // 可视化深度图，灰度图
        cv::Mat depth_vis;
        // 将深度值转换为 0-255 范围，假设最大深度为 4000mm
        image.convertTo(depth_vis, CV_8U, 255.0 / 4000.0); 
        image = depth_vis;
    }
    // 显示图像
    if (!image.empty()) {
        cv::resize(image, image, cv::Size(960, 540));
        cv::imshow(window_name, image);
        cv::moveWindow(window_name, 20, 100);
    }
}
void RealTimeDisplay::realTimeDisplay(k4a::device& device, RealTimeDisplayState State)
{
    k4a::capture capture;

    while (running.load()) 
    {
        if (device.get_capture(&capture, std::chrono::milliseconds(5000))) {
            // Get color image
            k4a::image color_image = capture.get_color_image();
            if (color_image && (State == RealTimeDisplayState::ONLY_COLOR || State == RealTimeDisplayState::COLOR_DEPTH)) {
                display_image(color_image, "Color Image");
            }
            else if(!color_image){
                std::cerr << "Failed to display a color image." << std::endl;
            }

            // Get depth image
            k4a::image depth_image = capture.get_depth_image();
            if (depth_image && (State == RealTimeDisplayState::ONLY_DEPTH || State == RealTimeDisplayState::COLOR_DEPTH)) {
                display_image(depth_image, "Depth Image");
            }
            else if(!depth_image){
                std::cerr << "Failed to display a depth image." << std::endl;
            }
            // Check for key press to exit
            if (cv::waitKey(30) >= 0) {
                running.store(false);
                break;
            }
        }
    }
}