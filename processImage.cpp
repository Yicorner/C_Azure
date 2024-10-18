#include "processImage.hpp"


void process_depth_image(int image_id) {
    // 定义深度图像尺寸
    const int depth_width = 640;
    const int depth_height = 576;

    // 构建文件名
    std::string filename = "depth_image_" + std::to_string(image_id);
    std::string input_file = "captures/" + filename + ".raw";

    // 打开原始深度数据文件
    std::ifstream file(input_file, std::ios::binary);
    if (!file) {
        std::cerr << "无法打开文件：" << input_file << std::endl;
        return;
    }

    // 计算预期的数据大小
    size_t expected_size = depth_width * depth_height * sizeof(uint16_t); // 每个像素2字节，uint16_t类型
    std::vector<uint16_t> buffer(depth_width * depth_height);

    // 读取原始深度数据
    file.read(reinterpret_cast<char*>(buffer.data()), expected_size);
    size_t read_size = file.gcount();

    // 检查读取的数据大小是否匹配
    if (read_size != expected_size) {
        std::cerr << "警告：预期数据大小为 " << expected_size << " 字节，但实际读取了 " << read_size << " 字节。" << std::endl;
        // 根据需要处理这个不匹配
    }

    // 关闭文件
    file.close();

    // 将数据转换为 OpenCV 的 Mat 对象
    cv::Mat depth_image(depth_height, depth_width, CV_16UC1, buffer.data());

    // 检查深度数据是否全部为零
    double min_depth, max_depth;
    cv::minMaxLoc(depth_image, &min_depth, &max_depth, nullptr, nullptr, depth_image > 0);

    if ((min_depth == 0 && max_depth == 0) || std::isnan(min_depth) || std::isnan(max_depth)) {
        std::cerr << "深度数据仅包含零值。" << std::endl;
        return;
    }

    // 将深度值转换为浮点型以便处理
    cv::Mat depth_in_meters;
    depth_image.convertTo(depth_in_meters, CV_32F);

    // 归一化深度值，除以 1000 并截断大于 1 的值
    depth_in_meters = depth_in_meters / 1000.0;
    cv::threshold(depth_in_meters, depth_in_meters, 1.0, 1.0, cv::THRESH_TRUNC);

    // 将深度值缩放到 0 - 255 的范围
    cv::Mat normalized_depth = depth_in_meters * 255.0;

    // 转换为 uint8 类型
    normalized_depth.convertTo(normalized_depth, CV_8UC1);

    // 创建一个 RGB 图像，每个通道都是归一化后的深度图像
    cv::Mat rgb_image;
    cv::cvtColor(normalized_depth, rgb_image, cv::COLOR_GRAY2BGR);

    // 保存图像为 PNG 文件
    std::string output_file = "captures/" + filename + ".png";
    if (!cv::imwrite(output_file, rgb_image)) {
        std::cerr << "无法保存图像到文件：" << output_file << std::endl;
        return;
    }

    std::cout << "深度图像成功保存为：" << output_file << std::endl;
}

void process_color_image(int image_id) {
    // 定义图像尺寸
    const int width = 1920;
    const int height = 1080;

    // 构建文件名
    std::string filename = "color_image_" + std::to_string(image_id);
    std::string input_file = "captures/" + filename + ".raw";

    // 打开原始数据文件
    std::ifstream file(input_file, std::ios::binary);
    if (!file) {
        std::cerr << "无法打开文件：" << input_file << std::endl;
        return;
    }

    // 计算预期的数据大小
    size_t expected_size = width * height * 4; // 4 字节每像素（RGBA）
    std::vector<unsigned char> buffer(expected_size);

    // 读取原始数据
    file.read(reinterpret_cast<char*>(buffer.data()), expected_size);
    size_t read_size = file.gcount();

    // 检查读取的数据大小是否匹配
    if (read_size != expected_size) {
        std::cerr << "警告：预期数据大小为 " << expected_size << "，但实际读取了 " << read_size << " 字节。" << std::endl;
        // 根据需要处理这个不匹配
    }

    // 将数据转换为 OpenCV 的 Mat 对象
    cv::Mat image(height, width, CV_8UC4, buffer.data());

    // 如果图像是倒置的，可以选择上下翻转
    // cv::flip(image, image, 0);

    // 保存图像为 PNG 文件
    std::string output_file = "captures/" + filename + ".png";
    if (!cv::imwrite(output_file, image)) {
        std::cerr << "无法保存图像到文件：" << output_file << std::endl;
        return;
    }

    std::cout << "图像成功保存为：" << output_file << std::endl;
}