#include "StaticFunction.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <fstream>
#include <vector>
// StaticFunction.cpp
MultiTimer& StaticFunction::timer = MultiTimer::getInstance();
void StaticFunction::process_depth_image(int image_id) {
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


void StaticFunction::process_depth_image_with_filename(std::string filename, std::string input_file) {
    // 定义深度图像尺寸
    const int depth_width = 640;
    const int depth_height = 576;

    // 构建文件名
    //std::string filename = "depth_image_" + std::to_string(image_id);
    //std::string input_file = "captures/" + filename + ".raw";

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
    std::string output_file = filename;
    if (!cv::imwrite(output_file, rgb_image)) {
        std::cerr << "无法保存图像到文件：" << output_file << std::endl;
        return;
    }

    std::cout << "深度图像成功保存为：" << output_file << std::endl;
}

void StaticFunction::process_color_image(int image_id) {
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

void StaticFunction::display_image_from_k4aimage(k4a::image& color_image, const std::string& window_name)
{
    /*
    * for debug
    *    just display the color_image from k4a::image
    */
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

        // 使用 OpenCV 显示图像
        cv::imshow(window_name, image);
        cv::waitKey(30);  // 等待 1 毫秒, 以便窗口刷新
    }
    else {
        std::cerr << "in work No valid color image to display!" << std::endl;
    }
}

// 定义评估拟合质量的函数 by hy
double StaticFunction::evaluate_fit_quality(const std::vector<cv::Point>& contour, const cv::RotatedRect& ellipse) {
    cv::Point2f center = ellipse.center;
    cv::Size2f axes = ellipse.size;
    float angle = ellipse.angle;
    // 将角度转换为弧度
    double angle_rad = angle * CV_PI / 180.0;
    double cos_angle = cos(angle_rad);
    double sin_angle = sin(angle_rad);
    double a = axes.width / 2.0;
    double b = axes.height / 2.0;
    double total_distance = 0.0;
    for (size_t i = 0; i < contour.size(); ++i) {
        double x = contour[i].x;
        double y = contour[i].y;
        double dx = x - center.x;
        double dy = y - center.y;
        double x_rot = dx * cos_angle + dy * sin_angle;
        double y_rot = -dx * sin_angle + dy * cos_angle;
        double distance = pow(x_rot / a, 2) + pow(y_rot / b, 2) - 1.0;
        total_distance += abs(distance);
    }
    double mean_distance = total_distance / contour.size() / contour.size();
    return mean_distance;
}

k4a::image StaticFunction::convert_file2image(const std::string& color_filename) {
    /*
     *  for debug
     *  so we don't need to implement a fast version.
     */
     // Load the image using OpenCV
    cv::Mat cv_image = cv::imread(color_filename, cv::IMREAD_UNCHANGED);
    if (cv_image.empty()) {
        std::cerr << "Failed to load image: " << color_filename << std::endl;
    }

    // Convert image to BGRA format if necessary
    if (cv_image.channels() == 3) {
        cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2BGRA);
    }
    else if (cv_image.channels() == 1) {
        cv::cvtColor(cv_image, cv_image, cv::COLOR_GRAY2BGRA);
    }
    else if (cv_image.channels() != 4) {
        std::cerr << "Unsupported number of channels: " << cv_image.channels() << std::endl;
    }

    // Calculate stride
    int stride_bytes = cv_image.cols * cv_image.elemSize();

    // Create k4a::image
    k4a::image color_image = k4a::image::create(
        K4A_IMAGE_FORMAT_COLOR_BGRA32,
        cv_image.cols,
        cv_image.rows,
        stride_bytes
    );

    // Copy data to k4a::image
    std::memcpy(color_image.get_buffer(), cv_image.data, cv_image.total() * cv_image.elemSize());

    // Obtain a reference if needed
    k4a::image& color_image_ref = color_image;
    return color_image_ref;
}

void StaticFunction::save_depth_image_in_raw(k4a::image& depth_image, const std::string& filename) {
    // filenamemust be raw file
    if (depth_image)
    {
        // Get depth image data
        uint16_t* depth_buffer = reinterpret_cast<uint16_t*>(depth_image.get_buffer());
        int depth_width = depth_image.get_width_pixels();
        int depth_height = depth_image.get_height_pixels();
        size_t depth_size = depth_image.get_size();
        // Save the depth image as a raw file
        std::ofstream depth_file(filename, std::ios::out | std::ios::binary);
        depth_file.write(reinterpret_cast<char*>(depth_buffer), depth_size);
        depth_file.close();
        std::cout << "depth raw Image saved successfully to " << filename << std::endl;
    }
    else {
        assert(0 && "depth image is invalid!");
    }
}
void StaticFunction::save_k4a_color_image_to_png(k4a::image& color_image, const std::string& filename) {
    // 检查图像是否有效
    if (!color_image.is_valid()) {
        std::cerr << "Invalid color image provided." << std::endl;
        return;
    }

    // 获取图像的宽度、高度和数据指针
    int width = color_image.get_width_pixels();
    int height = color_image.get_height_pixels();
    uint8_t* buffer = color_image.get_buffer();

    // 将 Azure Kinect 的 BGRA 图像转换为 OpenCV 的 Mat
    cv::Mat image(height, width, CV_8UC4, buffer);


    // 保存图像
    if (!cv::imwrite(filename, image)) {
        std::cerr << "Failed to save image to " << filename << std::endl;
    }
    else {
        std::cout << "Color Image saved successfully to " << filename << std::endl;
    }
}

void StaticFunction::save_body3Dlocation_list_in_txt_file(const std::string& filename, const std::vector<std::vector<float>>& body3Dlocation_list, const std::vector<std::pair<int, int>> bodylocation_list) {
    // 打开文件
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    // 输出bodylocation_list到文件
    file << "bodylocation_list:" << std::endl;
    for (size_t i = 0; i < bodylocation_list.size(); ++i) {
        file << "bodylocation_list[" << i << "]: " << bodylocation_list[i].first << " " << bodylocation_list[i].second << std::endl;
    }

    // 输出body3Dlocation_list到文件
    file << "body3Dlocation_list:" << std::endl;
    for (size_t i = 0; i < body3Dlocation_list.size(); ++i) {
        file << "body3Dlocation_list[" << i << "]: "
            << body3Dlocation_list[i][0] << " "
            << body3Dlocation_list[i][1] << " "
            << body3Dlocation_list[i][2] << std::endl;
    }
    // 关闭文件
    file.close();
    std::cout << "bodylocation_list与body3Dlocation_list数据已成功保存到文件: " << filename << std::endl;
}
void StaticFunction::save_mat_image(cv::Mat& mat, const std::string& filename) {
    // 检查图像是否有效
    if (mat.empty()) {
        std::cerr << "Invalid image provided." << std::endl;
        return;
    }
    // 保存图像
    if (!cv::imwrite(filename, mat)) {
        std::cerr << "Failed to save image to " << filename << std::endl;
    }
    else {
        std::cout << "color Image saved successfully to " << filename << std::endl;
    }
}



float StaticFunction::distance(std::vector<float>& p1, std::vector<float>& p2) {
    float dx = p1[0] - p2[0];
    float dy = p1[1] - p2[1];
    float dz = p1[2] - p2[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

void StaticFunction::display_image_from_filename(const std::string& color_filename, const std::string& depth_filename)
{
    /*
    * for debug
    *   just display the color_image and depth_image from png file.
    */
    cv::Mat color_image = cv::imread(color_filename, cv::IMREAD_COLOR);
    cv::Mat depth_image = cv::imread(depth_filename, cv::IMREAD_ANYDEPTH);

    // 显示图像
    if (!color_image.empty()) {
        cv::imshow("Color Image in work from " + color_filename, color_image);
        cv::waitKey(30);  // 等待 1 毫秒, 以便窗口刷新
    }
    else {
        std::cerr << "No valid color image to display!" << std::endl;
    }

    if (!depth_image.empty()) {
        cv::imshow("Depth Image in work from " + depth_filename, depth_image);
        cv::waitKey(30);  // 等待 1 毫秒, 以便窗口刷新
    }
    else {
        std::cerr << "No valid depth image to display!" << std::endl;
    }
}


void StaticFunction::removeInvalidLocations(std::vector<std::vector<float>>& body3Dlocation_list, const std::vector<float>& comp) {
    // 使用迭代器遍历并删除目标元素
    for (auto it = body3Dlocation_list.begin(); it != body3Dlocation_list.end(); ) {
        if (*it == comp) {
            it = body3Dlocation_list.erase(it);  // 删除元素并更新迭代器
        }
        else {
            ++it;  // 若不删除，移动到下一个元素
        }
    }
}


cv::Mat StaticFunction::k4aImageToCvMat(const k4a::image& k4aImage) {
    // 获取图像宽度、高度和格式
    int width = k4aImage.get_width_pixels();
    int height = k4aImage.get_height_pixels();
    k4a_image_format_t format = k4aImage.get_format();

    // 检查图像格式，并根据不同格式创建对应的 cv::Mat
    switch (format) {
    case K4A_IMAGE_FORMAT_COLOR_BGRA32: // BGRA 格式（每个像素 4 字节）
        return cv::Mat(height, width, CV_8UC4, (void*)k4aImage.get_buffer());

    case K4A_IMAGE_FORMAT_DEPTH16: // 16 位深度图像
        return cv::Mat(height, width, CV_16U, (void*)k4aImage.get_buffer());

    case K4A_IMAGE_FORMAT_IR16: // 16 位红外图像
        return cv::Mat(height, width, CV_16U, (void*)k4aImage.get_buffer());

    default:
        throw std::runtime_error("不支持的 k4a::image 格式");
    }
}


cv::Mat StaticFunction::addCone(const k4a::image& k4aImage) {
	timer.start("addCone");
    cv::Mat befres(1080, 1920, CV_8UC4, cv::Scalar(0, 0, 0, 0));
    cv::Mat ref = StaticFunction::k4aImageToCvMat(k4aImage);
    // 相机参数
    double fov_x = 92.75274; // 水平视场角，单位：度
    double fov_y = 61.11177; // 垂直视场角，单位：度
    int image_width = 1920;
    int image_height = 1080;
    // 计算焦距fx和fy
    double fx = (image_width / 2.0) / tan((fov_x * CV_PI / 180.0) / 2.0);
    double fy = (image_height / 2.0) / tan((fov_y * CV_PI / 180.0) / 2.0);

    // 主点坐标（假设在图像中心）
    double cx = image_width / 2.0;
    double cy = image_height / 2.0;

    // 构建相机内参矩阵
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1);

    // 假设无畸变
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

    // 转换为灰度图
    cv::Mat gray;
    cv::cvtColor(ref, gray, cv::COLOR_BGR2GRAY);

    // 定义ArUco字典
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    // 定义检测参数
    cv::aruco::DetectorParameters parameters;

    // 检测标记
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    cv::aruco::ArucoDetector detector(dictionary, parameters);

    detector.detectMarkers(gray, corners, ids, rejected);


    double coneHeight = 0.30;  // 圆锥体的高度
    double coneRadius = 0.015; // 圆锥体底面半径


    // 如果检测到标记
    if (ids.size() > 0)
    {
        // 假设标记的实际边长为0.05米（5厘米），请根据实际情况修改
        float markerLength = 0.05f;

        // 估计姿态
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

        // 绘制结果并输出
        for (size_t i = 0; i < ids.size(); i++)
        {
            // 绘制标记边框
            cv::cvtColor(befres, befres, cv::COLOR_BGRA2BGR);
            std::cout << befres.channels() << " " << befres.total() << " " << befres.size << std::endl;
            cv::aruco::drawDetectedMarkers(befres, corners, ids);

            // 绘制坐标轴
            // 如果aruco命名空间下没有drawAxis，可以使用cv::drawFrameAxes
            cv::drawFrameAxes(befres, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);

            //rvecs[i][0] *= -1;
            //rvecs[i][1] *= -1;
            //rvecs[i][2] *= -1;
            //double angle = cv::norm(rvecs[i]);  // 获取旋转角度（弧度）
            //double new_angle = angle + CV_PI;  // 增加 180 度
            //rvecs[i] = rvecs[i] * (new_angle / angle);
            //std::cout << rvecs[i] << std::endl;

            // 输出标记ID、坐标和朝向
            /*std::cout << "标记ID: " << ids[i] << std::endl;
            std::cout << "平移向量 tvec: [" << tvecs[i][0] << ", " << tvecs[i][1] << ", " << tvecs[i][2] << "]" << std::endl;
            std::cout << "旋转向量 rvec: [" << rvecs[i][0] << ", " << rvecs[i][1] << ", " << rvecs[i][2] << "]" << std::endl;
            std::cout << std::endl;*/

            // 将旋转向量转换为旋转矩阵
            cv::Mat R;
            cv::Rodrigues(rvecs[i], R);
            //std::cout << R.at<double>(0, 0) << " " << R.at<double>(0, 1) << " " << R.at<double>(0, 2) << std::endl;

            // 创建旋转平移矩阵
            cv::Mat Rt = cv::Mat::zeros(3, 4, CV_64F);
            R.copyTo(Rt(cv::Rect(0, 0, 3, 3)));  // 将旋转矩阵 R 复制到 Rt 的前 3 列

            // 将平移向量 tvec 转换为 cv::Mat 并赋值给 Rt 的最后一列
            cv::Mat tvecMat = (cv::Mat_<double>(3, 1) << tvecs[i][0], tvecs[i][1], tvecs[i][2]);
            tvecMat.copyTo(Rt.col(3));  // 确保 tvecMat 格式与 Rt 的第四列匹配


            // 创建4x4齐次变换矩阵
            cv::Mat transformMatrix = cv::Mat::eye(4, 4, CV_64F);  // 创建一个4x4单位矩阵
            Rt.copyTo(transformMatrix(cv::Rect(0, 0, 4, 3)));  // 将3x4矩阵复制到4x4矩阵的前3行


            // 打印旋转平移矩阵
            //std::cout << "4x4旋转平移矩阵: " << std::endl << transformMatrix << std::endl;




            cv::Vec3d rvec = rvecs[i];
            cv::Vec3d tvec = tvecs[i];


            // 圆锥体顶点位置，沿 z 轴方向延伸 coneHeight
            cv::Point3d coneTip = cv::Point3d(tvec[0], tvec[1], tvec[2]) - //!!!
                cv::Point3d(R.at<double>(0, 2) * coneHeight,
                    R.at<double>(1, 2) * coneHeight,
                    R.at<double>(2, 2) * coneHeight);
            /*std::cout << R << std::endl;
            std::cout << "圆锥高度：" << coneHeight << std::endl;
            std::cout << "z轴延伸方向：" << R.at<double>(0, 2) << " " << R.at<double>(1, 2) << " " << R.at<double>(2, 2) << std::endl;
            std::cout << "圆锥轴向量：" << R.at<double>(0, 2) * coneHeight << " " << R.at<double>(1, 2) * coneHeight << " " << R.at<double>(2, 2) * coneHeight << std::endl;
            std::cout << "coneTip: " << coneTip << std::endl;*/
            if (coneTip.z <= 0) coneTip.z = 0.000001;

            std::vector<cv::Point3d> points3Dtmp = { cv::Point3d(tvec[0], tvec[1], tvec[2]), cv::Point3d(coneTip) };
            std::vector<cv::Point2d> points2Dtmp;
            cv::projectPoints(points3Dtmp, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), cameraMatrix, distCoeffs, points2Dtmp);
            //cv::line(image, points2Dtmp[1], points2Dtmp[0], cv::Scalar(255, 255, 0), 4);
            std::cout << points2Dtmp[0] << " " << points2Dtmp[1] << std::endl;

            // 生成底面圆上的点
            std::vector<cv::Point3d> coneBasePoints;
            int numPoints = 32; // 底面圆上的点数
            for (int j = 0; j < numPoints; j++)
            {
                double angle = 2 * CV_PI * j / numPoints;
                double x = coneRadius * cos(angle);
                double y = coneRadius * sin(angle);

                // 将圆上的点从局部坐标系转换到全局坐标系
                cv::Point3d pointOnCircle = cv::Point3d(tvec[0], tvec[1], tvec[2]) +
                    cv::Point3d(R.at<double>(0, 0) * x + R.at<double>(0, 1) * y,
                        R.at<double>(1, 0) * x + R.at<double>(1, 1) * y,
                        R.at<double>(2, 0) * x + R.at<double>(2, 1) * y);

                coneBasePoints.push_back(pointOnCircle);
            }

            // 将 3D 点投影到 2D 图像平面
            std::vector<cv::Point3d> points3D = coneBasePoints;
            points3D.push_back(coneTip); // 将顶点添加到投影点
            std::vector<cv::Point2d> points2D;
            cv::projectPoints(points3D, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), cameraMatrix, distCoeffs, points2D);

            // 绘制底面圆
            /*for (size_t j = 0; j < numPoints; j++)
            {
                cv::line(image, points2D[j], points2D[(j + 1) % numPoints], cv::Scalar(0, 255, 0), 2);
            }*/

            // 绘制从顶点到底面圆的线段，形成圆锥体
            for (size_t j = 0; j < numPoints; j++)
            {
                cv::line(befres, points2D[j], points2D.back(), cv::Scalar(0, 0, 255), 1);
            }
        }
    }
    else
    {
        std::cout << "未检测到ArUco标记。" << std::endl;
    }
	timer.stop("addCone");
    return befres;
}

// 计算向量 AB 和 BC 的二维叉积
float StaticFunction::crossProduct2D(const std::vector<float>& A, const std::vector<float>& B, const std::vector<float>& C) {
    return (B[0] - A[0]) * (C[1] - A[1]) - (B[1] - A[1]) * (C[0] - A[0]);
}

// 判断两个三角形的二维顺序是否一致
bool StaticFunction::isSameOrder(const std::vector<float>& chest1, const std::vector<float>& chest2, const std::vector<float>& chest3,
    const std::vector<float>& a, const std::vector<float>& b, const std::vector<float>& c) {
    // 计算两个三角形的二维叉积
    float crossChest = StaticFunction::crossProduct2D(chest1, chest2, chest3);
    float crossABC = StaticFunction::crossProduct2D(a, b, c);

    // 判断两个叉积的符号是否相同，正负相同即顺序一致
    return (crossChest * crossABC > 0);
}