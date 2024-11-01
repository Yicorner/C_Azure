#include <conio.h>  // Windows下用于非阻塞式获取键盘输入
#include <chrono>
#include <filesystem>
#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <fstream>  // 需要包含这个头文件
#include <locale>
#include <codecvt>
#include "GetCords.hpp"
#include "realTimeDisplay.hpp"
#include "getSample.hpp"
#include "work.hpp"
#include "AllEnum.hpp"


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
    std::ofstream error_file("error_log.txt");
    std::cerr.rdbuf(error_file.rdbuf());

	// 将 std::cout 重定向到文件
    std::ofstream file("normal_log.txt", std::ios::out | std::ios::binary);
    std::streambuf* original_cout = std::cout.rdbuf();
    std::cout.rdbuf(file.rdbuf());



    // Open Kinect device
    k4a::device device;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	change_device_config(device, config);
    device.start_cameras(&config);

	// realtimedisplay thread
	RealTimeDisplayState State = RealTimeDisplayState::ONLY_COLOR;
	RealTimeDisplay realtimedisplay;
    std::thread realTimeDisplay_thread(std::bind(&RealTimeDisplay::realTimeDisplay, &realtimedisplay, std::ref(device), std::ref(State)));

	// Infact, I don't need the object of getsample, just take it as an meanningless argumrnt and omit it.
    GetSample getsample;
	Work work(device, config);
    work.run(getsample);


    realTimeDisplay_thread.join();

    device.stop_cameras();
    device.close();


    std::cout.rdbuf(original_cout);
    file.close();
    error_file.close();
    return 0;
}


