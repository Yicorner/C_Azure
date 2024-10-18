#include <k4a/k4a.hpp>
#include <conio.h>  // Windows下用于非阻塞式获取键盘输入
#include <chrono>
#include <thread>
#include "processImage.hpp"
#include "GetCords.hpp"
#include <opencv2/highgui.hpp>
#include <filesystem>
#include <iostream>
void get_sample(k4a::device& device, k4a_device_configuration_t& config, int image_id) {
    k4a::capture capture;
	std::cout << "in get_sample 捕获图像中..." << std::endl;
    if (device.get_capture(&capture, std::chrono::milliseconds(5000)))
    {
        // 从捕获的数据中获取RGB图像
        k4a::image color_image = capture.get_color_image();

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
        k4a::image depth_image = capture.get_depth_image();

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
// //定义全局线程对象，或者在需要的地方创建线程
//std::thread t;
//void preview_image_opencv() {
//    // 打开默认相机（设备ID为0）
//    cv::VideoCapture cap(0);
//
//    if (!cap.isOpened()) {
//        std::cerr << "无法打开相机。" << std::endl;
//        return;
//    }
//    else {
//        std::cout << "相机已打开。" << std::endl;
//    }
//
//    // 设置相机分辨率（可选）
//    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
//    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
//
//    // 初始化窗口
//    const std::string window_name = "Camera Preview";
//
//    cv::Mat frame;
//    while (1)
//    {
//        cap >> frame;
//        if (frame.empty())
//            break;
//        cv::imshow("video", frame);
//        if (cv::waitKey(200) > 0)//按下任意键退出摄像头
//            break;
//    }
//    cap.release();
//    cv::destroyAllWindows();//关闭所有窗
//}
//void run_preview() {
//    // 在独立线程中运行 preview_image_opencv
//    t = std::thread(preview_image_opencv);
//}
int main()
{

    try
    {
        //多线程运行preview_image_opencv
		//run_preview();
        // Open the device
        k4a::device device = k4a::device::open(0);
        // Configure the device
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.color_resolution = K4A_COLOR_RESOLUTION_1080P;   // 1920x1080
        config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;       // 640x576
        config.camera_fps = K4A_FRAMES_PER_SECOND_5;
        config.synchronized_images_only = true;                 // Ensure synchronized captures
        config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;

        // Start the cameras
        device.start_cameras(&config);
        //get_sample();
        std::cout << "程序已启动。按 'G' 键捕获图像，按 'Q' 键退出程序。" << std::endl;



        
        
        bool running = true;
        int image_id = 0;
        while (running)
        {   
            if (_kbhit())
            {
                // 获取按下的键
                char ch = _getch();

                if (ch == 'G' || ch == 'g')
                {
                    image_id++;
                    get_sample(device, config, image_id);
                    process_color_image(image_id);
                    process_depth_image(image_id);
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
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
			std::cout << "正在运行" << std::endl;
        }
        // 获取colorspace坐标
		//get_cords(x, y, image_id);
    }
    catch (const k4a::error& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }



    //// 确保主线程等待子线程结束
    //if (t.joinable()) {
    //    t.join();
    //}
    return 0;
}
