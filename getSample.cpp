// Description: get sample from kinect device
#include "getSample.hpp"
#include "processImage.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <conio.h>  // Windows下用于非阻塞式获取键盘输入
#include <thread>
// 定义静态成员变量
//std::mutex GetSample::color_mtx;
//std::mutex GetSample::depth_mtx;

void GetSample::get_sample(k4a::device& device, k4a_device_configuration_t& config, int image_id) {
    /*
    get image sample with native kinect sdk method*/
    k4a::capture capture;
    std::cout << "in get_sample 捕获图像中..." << std::endl;
    if (device.get_capture(&capture, std::chrono::milliseconds(5000)))
    {
        // 从捕获的数据中获取RGB图像
        color_mtx.lock();
        color_image = capture.get_color_image();
        color_mtx.unlock();
        if (color_image)
        {
            std::cout << "Successfully captured a color image!" << std::endl;

            // 获取图像数据
            uint8_t* buffer = color_image.get_buffer();
            int width = color_image.get_width_pixels();
            int height = color_image.get_height_pixels();
            size_t size = color_image.get_size();
            printf("Image width: %4d, height: %4d, size: %zu\n", width, height, size);
            // 保存图像为文件（如保存为原始RGB数据）
            std::string filename = "color_image_" + std::to_string(image_id) + ".raw";
            std::ofstream file("captures/" + filename, std::ios::out | std::ios::binary);
            file.write(reinterpret_cast<char*>(buffer), size);
            file.close();

        }
        else
        {
            std::cout << "Failed to capture a color image." << std::endl;
        }
        // 从捕获的数据中获取Depth图像
		depth_mtx.lock();
        depth_image = capture.get_depth_image();
		depth_mtx.unlock();

        if (depth_image)
        {
            std::cout << "Successfully captured a depth image!" << std::endl;

            // Get depth image data
            uint16_t* depth_buffer = reinterpret_cast<uint16_t*>(depth_image.get_buffer());
            int depth_width = depth_image.get_width_pixels();
            int depth_height = depth_image.get_height_pixels();
            size_t depth_size = depth_image.get_size();
            printf("Depth Image width: %4d, height: %4d, size: %zu\n", depth_width, depth_height, depth_size);

            // Save the depth image as a raw file
            std::string filename = "depth_image_" + std::to_string(image_id) + ".raw";
            std::ofstream depth_file("captures/" + filename, std::ios::out | std::ios::binary);
            depth_file.write(reinterpret_cast<char*>(depth_buffer), depth_size);
            depth_file.close();
        }
        else
        {
            std::cout << "Failed to capture a depth image." << std::endl;
        }
    }
}
void GetSample::display_current_image() {
    if (color_image) {
        // 获取图像的缓冲区指针
        uint8_t* buffer = color_image.get_buffer();

        // 获取图像的宽度、高度和步幅（每行的字节数）
        int width = color_image.get_width_pixels();
        int height = color_image.get_height_pixels();
        int stride = color_image.get_stride_bytes();

        // 将 Kinect 的图像转换为 OpenCV 格式的 cv::Mat
        // 假设 color_image 是 RGB 图像（每个像素有3个字节）
        cv::Mat image(height, width, CV_8UC4, buffer, stride);  // 假设为 BGRA/RGBA

        // 转换为 BGR 格式，OpenCV 使用 BGR 顺序
        cv::Mat image_bgr;
        cv::cvtColor(image, image_bgr, cv::COLOR_BGRA2BGR);

        // 使用 OpenCV 显示图像
        cv::imshow("Color Image in GetSample", image_bgr);

        // 等待键盘输入以关闭窗口
        cv::waitKey(30);  // 等待 1 毫秒, 以便窗口刷新
    }
    else {
        std::cerr << "in getsample No valid color image to display!" << std::endl;
    }
}
void GetSample::get_sample_start(k4a::device& device, k4a_device_configuration_t& config) {
    /*
    the initiation before get_sample, start_cameras. but we can't start camera twice
    */
    try
    {
        //get_sample();
        std::cout << "程序已启动。按 'G' 键捕获图像，按 'Q' 键退出程序。" << std::endl;
        bool running = true;
        while (running)
        {
            if (_kbhit())
            {
                // 获取按下的键
                char ch = _getch();

                if (ch == 'G' || ch == 'g')
                {
                    count++;
                    if (count % 2 == 1) {
                        image_id = count / 2 + 1;
                        get_sample(device, config, image_id);
                        process_color_image(image_id);
                        process_depth_image(image_id);
                    }
                    else {
                        std::cout << "已暂停" << std::endl;
                    }
                }
                else if (ch == 'Q' || ch == 'q')
                {
                    std::cout << "退出程序。" << std::endl;
                    running = false;
                }
                else
                {
                    std::cout << "未知的按键：" << ch << std::endl;
                    std::cout << "按 'G' 键捕获图像，按 'Q' 键退出程序。" << std::endl;
                }
            }
            // 休眠一段时间，避免占用过多CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            std::cout << "get_sample_start 正在运行" << std::endl;
			std::cout << "start display static image in getsample" << std::endl;
            display_current_image();
        }
        // 获取colorspace坐标
        //get_cords(x, y, image_id);
    }
    catch (const k4a::error& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}