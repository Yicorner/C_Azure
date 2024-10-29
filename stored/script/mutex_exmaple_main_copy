#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <mutex>

// 全局变量
std::mutex img_mutex; // 互斥锁，用于保护共享数据
cv::Mat color_image_mat;
cv::Mat depth_image_mat;
bool stop_display = false; // 用于控制显示线程的退出

void display_images()
{
    while (!stop_display)
    {
        std::lock_guard<std::mutex> lock(img_mutex); // 确保图像数据访问时是线程安全的
        if (!color_image_mat.empty()) {
            cv::imshow("RGBA Image", color_image_mat);
        }
        if (!depth_image_mat.empty()) {
            cv::imshow("Depth Image", depth_image_mat);
        }
        std::cout << "display_images 正在运行" << std::endl;
        // 等待 30ms，若有键盘输入则退出
        if (cv::waitKey(30) >= 0) {
            stop_display = true;
        }
    }
}

void process_images(k4a::capture& capture)
{
    k4a::image color_image = capture.get_color_image();
    k4a::image depth_image = capture.get_depth_image();

    if (color_image) {
        int width = color_image.get_width_pixels();
        int height = color_image.get_height_pixels();
        const uint8_t* buffer = color_image.get_buffer();
        std::lock_guard<std::mutex> lock(img_mutex); // 确保图像数据访问时是线程安全的
        color_image_mat = cv::Mat(height, width, CV_8UC4, (void*)buffer).clone(); // 复制图像
        std::cout << "color_image_mat" << std::endl;
    }

    if (depth_image) {
        int width = depth_image.get_width_pixels();
        int height = depth_image.get_height_pixels();
        const uint16_t* buffer = reinterpret_cast<const uint16_t*>(depth_image.get_buffer());
        std::lock_guard<std::mutex> lock(img_mutex); // 确保图像数据访问时是线程安全的
        cv::Mat depth_image_raw = cv::Mat(height, width, CV_16U, (void*)buffer).clone();

        // 转换深度图像为 8 位灰度图用于显示
        depth_image_raw.convertTo(depth_image_mat, CV_8U, 255.0 / 4000.0); // 假设最大深度为 4000mm
        std::cout << "depth_image_mat" << std::endl;

    }
}

int main()
{
    // 打开Kinect设备
    k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);

    // 启动设备并设置深度和彩色模式
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;                 // Ensure synchronized captures
    config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    device.start_cameras(&config);

    // 启动图像显示线程
    std::thread display_thread(display_images);

    k4a::capture capture;
    std::cout << "Capturing images..." << std::endl;

    while (!stop_display) // 实时捕获图像
    {
        if (device.get_capture(&capture, std::chrono::milliseconds(5000))) {
            process_images(capture);
        }
        // 按下任意键退出
        if (cv::waitKey(30) >= 0) {
            break;
        }
    }
    // 等待显示线程退出
    display_thread.join();

    // 停止相机并关闭设备
    device.stop_cameras();
    device.close();

    return 0;
}