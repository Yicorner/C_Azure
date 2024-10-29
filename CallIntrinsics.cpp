#include "CalIntrinsics.hpp"
#include <iostream>

void call_intrinsics(k4a::device& device, k4a_device_configuration_t& config) {

    // 获取设备校准数据
    k4a::calibration calibration = device.get_calibration(config.depth_mode, config.color_resolution);
    // 获取彩色相机的内参
    k4a_calibration_camera_t color_camera_calibration = calibration.color_camera_calibration;

    // 获取彩色相机的内参参数
    k4a_calibration_intrinsics_t intrinsics = color_camera_calibration.intrinsics;
    k4a_calibration_intrinsic_parameters_t parameters = intrinsics.parameters;

    // 打印内参数据
    std::cout << "Color Camera Intrinsics:" << std::endl;
    std::cout << "cx (Principal Point x): " << parameters.param.cx << std::endl;
    std::cout << "cy (Principal Point y): " << parameters.param.cy << std::endl;
    std::cout << "fx (Focal Length x): " << parameters.param.fx << std::endl;
    std::cout << "fy (Focal Length y): " << parameters.param.fy << std::endl;

    std::cout << "Radial Distortion Coefficients:" << std::endl;
    std::cout << "k1: " << parameters.param.k1 << std::endl;
    std::cout << "k2: " << parameters.param.k2 << std::endl;
    std::cout << "k3: " << parameters.param.k3 << std::endl;
    std::cout << "k4: " << parameters.param.k4 << std::endl;
    std::cout << "k5: " << parameters.param.k5 << std::endl;
    std::cout << "k6: " << parameters.param.k6 << std::endl;

    std::cout << "Tangential Distortion Coefficients:" << std::endl;
    std::cout << "p1: " << parameters.param.p1 << std::endl;
    std::cout << "p2: " << parameters.param.p2 << std::endl;

    //std::cout << "Metric Radius: " << parameters.param.metric_radius << std::endl;
}