#include "realTimeDisplay.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

void RealTimeDisplay::display_image(k4a::image& k4a_image, std::string window_name)
{
    // ��ȡͼ��Ŀ�ȡ��߶Ⱥ�ָ��
    int width = k4a_image.get_width_pixels();
    int height = k4a_image.get_height_pixels();
    const uint8_t* buffer = k4a_image.get_buffer();

    // ���ͼ���ʽ��������Ҫ����ת��
    cv::Mat image;
    if (k4a_image.get_format() == K4A_IMAGE_FORMAT_COLOR_BGRA32) {
        // ����� BGRA32 ��ʽ��RGBA��������ת��Ϊ OpenCV �� BGRA ͼ��
        image = cv::Mat(height, width, CV_8UC4, (void*)buffer);
    }
    else if (k4a_image.get_format() == K4A_IMAGE_FORMAT_DEPTH16) {
        // ��������ͼ��ֱ��ʹ�� 16 λ�޷�����������
        image = cv::Mat(height, width, CV_16U, (void*)buffer);
        // ���ӻ����ͼ���Ҷ�ͼ
        cv::Mat depth_vis;
        // �����ֵת��Ϊ 0-255 ��Χ������������Ϊ 4000mm
        image.convertTo(depth_vis, CV_8U, 255.0 / 4000.0); 
        image = depth_vis;
    }
    // ��ʾͼ��
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
            else {
                std::cerr << "Failed to display a color image." << std::endl;
            }

            // Get depth image
            k4a::image depth_image = capture.get_depth_image();
            if (depth_image && (State == RealTimeDisplayState::ONLY_DEPTH || State == RealTimeDisplayState::COLOR_DEPTH)) {
                display_image(depth_image, "Depth Image");
            }
            else {
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