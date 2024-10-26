#include <conio.h>  // Windows下用于非阻塞式获取键盘输入
#include <chrono>
#include <filesystem>
#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <fstream>  // 需要包含这个头文件
#include "processImage.hpp"
#include "GetCords.hpp"
#include "CalIntrinsics.hpp"
#include "realTimeDisplay.hpp"
#include "getSample.hpp"


void change_device_config(k4a::device& device, k4a_device_configuration_t& config) {
    device = k4a::device::open(0);
	// Configure the device
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_1080P;   // 1920x1080
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;       // 640x576
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.synchronized_images_only = true;                 // Ensure synchronized captures
	config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
}


int main()
{
    // Open Kinect device
    k4a::device device;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	change_device_config(device, config);
    device.start_cameras(&config);


    GetSample getsample;
	RealTimeDisplay realTimeDisplay;

    std::thread realTimeDisplay_thread(std::bind(&RealTimeDisplay::realTimeDisplay, &realTimeDisplay, std::ref(device)));
    std::thread capture_thread(std::bind(&GetSample::get_sample_start, &getsample, std::ref(device), std::ref(config)));


    // Main thread can perform other tasks here
    std::cout << "Main thread is doing other work..." << std::endl;
    // Example main thread tasks
    for (int i = 0; i < 3; ++i) {
        std::cout << "Main thread working: " << i << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }


    realTimeDisplay_thread.join();
	capture_thread.join();
    device.stop_cameras();
    device.close();

    return 0;
}

