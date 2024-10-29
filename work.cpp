#include "work.hpp"
#include "GetCords.hpp"
#include "RendererGUI.h"
#include "AllEnum.hpp"
#include "stb_image_write.h"
#include "processImage.hpp"

#include <iostream>
#include <k4a/k4a.h>
#include <opencv2/opencv.hpp>
#include <fstream>

void Work::run(GetSample& sample)
{
	// inital a object of RendererGUI for volume rendering
    RendererGUI vr(1440, 810, "Volume-Renderer");
    vr.setShaderAndData();
	// inital a object of k4a::capture for capture both color and depth iamge
    k4a::capture capture;
	// this id is used to save the image and txt file to debug
    while (1) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (state == FROM_FILE) {
            /*
				调试状态，从文件中读取图片，然后进行处理。
            */
            body3Dlocation_list.clear();
            const std::string color_filename = "D:\\data\\project\\VisualStudio\\C_Azure\\FROM_FILE\\color_5.png";
			const std::string depth_filename = "D:\\data\\project\\VisualStudio\\C_Azure\\FROM_FILE\\depth_5.raw";
            // get body3Dlocation_list
            k4a::image color_image = convert_file2image(color_filename);
			get_body_location(color_image);
			for (int i = 0; i < bodylocation_list.size(); i++) {
				get_cords_with_filename(*device, *config, bodylocation_list[i].first, bodylocation_list[i].second, depth_filename, body3Dlocation_list);
			}
            resort_3D_bodylocation_list();
            // the color image of color_filename is the abdomen of the patient
            bodylocation = BodyLocation::ABDOMEN;
			// start volume rendering
            vr.run_only_render(bodylocation, body3Dlocation_list, 80, 255, 0.3f);
            // get pointer which point to the CT volume render result
            vr.run_only_image_content();
            unsigned char* CT = vr.volren.img_data_from_core;
            // blend the color image and the CT volume render result to get the final blend result in variant result
            getResult(CT, color_image, vr.volren.framebuffer_size.x, vr.volren.framebuffer_size.y);
            //show final blend result
            cv::Mat resized_result;
            cv::resize(result, resized_result, cv::Size(960, 540));
            cv::imshow("Blended Image", resized_result);
            cv::moveWindow("Blended Image", 1000, 100);
            cv::waitKey(30);
        }
        else if (state == DYNAMIC) {
            if ((*device).get_capture(&capture, std::chrono::milliseconds(5000))) {
                // Get color image
                k4a::image color_image = capture.get_color_image();
                // Get depth image
                k4a::image depth_image = capture.get_depth_image();
                if (color_image && depth_image) {
                    body3Dlocation_list.clear();
                    get_body_location(color_image);
                    for (int i = 0; i < bodylocation_list.size(); i++) {
						// from pixel cordinates in color_image to 3D cordinates in color_image space cordinates
                       get_cords_with_depth_image(*device, *config, bodylocation_list[i].first, bodylocation_list[i].second, depth_image, body3Dlocation_list);
                    }
                    int detect_state = resort_3D_bodylocation_list();
                    if (detect_state == 1) {
                        save_file_id++;
						// save body3Dlocation_list in txt file
                        save_body3Dlocation_list_in_txt_file("D:/data/project/VisualStudio/C_Azure/verify_color_image/bodylocation_" + std::to_string(save_file_id) + ".txt", body3Dlocation_list, bodylocation_list);
						// Save color image
                        save_k4a_color_image_to_png(color_image, "D:/data/project/VisualStudio/C_Azure/verify_color_image/color_" + std::to_string(save_file_id) + ".png");
						// Save depth image in raw
						save_depth_image(depth_image, "D:/data/project/VisualStudio/C_Azure/verify_color_image/depth_" + std::to_string(save_file_id) + ".raw");
						// shift depth raw to png for visualization
                        process_depth_image_with_filename("D:/data/project/VisualStudio/C_Azure/verify_color_image/depth_" + std::to_string(save_file_id) + ".png", "D:/data/project/VisualStudio/C_Azure/verify_color_image/depth_" + std::to_string(save_file_id) + ".raw");
						// start volume rendering
                        bodylocation = BodyLocation::ABDOMEN;
                        vr.run_only_render(bodylocation, body3Dlocation_list, 80, 255, 0.4f);
						vr.run_only_image_content();
                        // get pointer which point to the CT volume render result
						unsigned char* CT = vr.volren.img_data_from_core;
						// blend the color image and the CT volume render result to get the final blend result in variant result
						getResult(CT, color_image, vr.volren.framebuffer_size.x, vr.volren.framebuffer_size.y);
						//show final blend result
						cv::Mat resized_result;
                        cv::resize(result, resized_result, cv::Size(960, 540));
						cv::imshow("Blended Image", resized_result);
                        cv::moveWindow("Blended Image", 1000, 100);
						cv::waitKey(30);
                        // save final blend result
						save_mat_image(result, "D:/data/project/VisualStudio/C_Azure/verify_color_image/result_" + std::to_string(save_file_id) + ".png");

						/*
						* the code below for debug to verify the correctness of CT
						* stbi_flip_vertically_on_write(1);
						* std::string fn = "D:/data/project/VisualStudio/C_Azure/rendered-image/2.png";
						* int stride = (vr.volren.framebuffer_size.x % 4) + (vr.volren.framebuffer_size.x * 3);
						* bool status = stbi_write_png(fn.c_str(), vr.volren.framebuffer_size.x, vr.volren.framebuffer_size.y, 3, CT, stride);
						*/
						// save CT volume render image
                        stbi_flip_vertically_on_write(1);
                        std::string fn = "D:/data/project/VisualStudio/C_Azure/verify_color_image/CT_" + std::to_string(save_file_id) + ".png";
                        int stride = (vr.volren.framebuffer_size.x % 4) + (vr.volren.framebuffer_size.x * 3);
                        bool status = stbi_write_png(fn.c_str(), vr.volren.framebuffer_size.x, vr.volren.framebuffer_size.y, 3, CT, stride);
                    }
                    else {
                        std::cout << "未识别到三个点，不进行处理" << std::endl;
                    }
                }
            }
        }
        else {
            assert(0 && "other state remain to implement");
        }
    }
}

void Work::save_body3Dlocation_list_in_txt_file(const std::string& filename, const std::vector<std::vector<float>>& body3Dlocation_list, const std::vector<std::pair<int, int>> bodylocation_list) {
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
void Work::save_mat_image(cv::Mat& mat, const std::string& filename) {
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

void Work::save_depth_image(k4a::image& depth_image, const std::string& filename) {
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
void Work::save_k4a_color_image_to_png(k4a::image& color_image, const std::string& filename) {
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

float Work::distance(std::vector<float>& p1, std::vector<float>& p2) {
    float dx = p1[0] - p2[0];
    float dy = p1[1] - p2[1];
    float dz = p1[2] - p2[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

int Work::resort_3D_bodylocation_list() {
	/*
		判断3D_bodylocation_list的合法性。返回0代表不合法，返回1代表合法。
	*/
    using namespace std;
    if(body3Dlocation_list.size() == 3){
		// 如果我们已经规定了体数据标点与拍摄角度，为了对应体数据标点与拍摄角度，我们需要对body3Dlocation_list进行排序
        // 目的是让其x降序排序，而body3Dlocation_list本身是正序的，所以这里需要反转一下
        std::reverse(body3Dlocation_list.begin(), body3Dlocation_list.end());
        return 1;
    }else {
        return 0;
    }

    // Define chest1, chest2, chest3
    vector<float> chest1 = { -0.211112f, -0.25f, 0.161112f };
    vector<float> chest2 = { -0.281456f, -0.227778f, 0.11111f };
    vector<float> chest3 = { -0.177778f, -0.244444f, 0.047222f };

    // Compute the sides of the chest triangle
    float chest_side1 = distance(chest1, chest2); // between chest1 and chest2
    float chest_side2 = distance(chest2, chest3); // between chest2 and chest3
    float chest_side3 = distance(chest3, chest1); // between chest3 and chest1

    // Store the chest sides in a vector for easier comparison
    vector<float> chest_sides = { chest_side1, chest_side2, chest_side3 };

    float tolerance = 0.05f; // Allowable relative deviation (10%)

    size_t n = body3Dlocation_list.size();

    // Iterate over all combinations of three distinct points
    for (size_t i = 0; i < n - 2; ++i) {
        for (size_t j = i + 1; j < n - 1; ++j) {
            for (size_t k = j + 1; k < n; ++k) {
                vector<vector<float>> points = {
                    body3Dlocation_list[i],
                    body3Dlocation_list[j],
                    body3Dlocation_list[k]
                };

                vector<int> perm = { 0, 1, 2 };
                bool match_found = false;

                // Consider all permutations of the three points
                do {
                    vector<float>& a = points[perm[0]];
                    vector<float>& b = points[perm[1]];
                    vector<float>& c = points[perm[2]];

                    // Compute sides of triangle abc in the same order as chest_sides
                    float side1 = distance(a, b); // corresponds to chest_side1
                    float side2 = distance(b, c); // corresponds to chest_side2
                    float side3 = distance(c, a); // corresponds to chest_side3

                    // Store the sides in order
                    vector<float> body_sides = { side1, side2, side3 };

                    // Compute scaling factors for corresponding sides
                    float s1 = body_sides[0] / chest_sides[0];
                    float s2 = body_sides[1] / chest_sides[1];
                    float s3 = body_sides[2] / chest_sides[2];

                    // Compute mean scaling factor
                    float s_mean = (s1 + s2 + s3) / 3.0f;

                    // Compute relative deviations
                    float dev1 = fabs(s1 - s_mean) / s_mean;
                    float dev2 = fabs(s2 - s_mean) / s_mean;
                    float dev3 = fabs(s3 - s_mean) / s_mean;

                    float max_dev = max({ dev1, dev2, dev3 });

                    if (max_dev < tolerance) {
                        // Found similar triangle with this permutation
                        body3Dlocation_list = { a, b, c };
                        match_found = true;
                        break;
                    }
                } while (next_permutation(perm.begin(), perm.end()));

                if (match_found) {
                    return 1;
                }
            }
        }
    }

    // No similar triangle found
    body3Dlocation_list.clear();
    return 0;

}

void Work::display_image_from_filename(const std::string& color_filename, const std::string& depth_filename)
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

void Work::display_image_from_k4aimage(k4a::image& color_image)
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
        cv::imshow("k4a Color Image in work ", image);
		cv::waitKey(30);  // 等待 1 毫秒, 以便窗口刷新
    }
    else {
        std::cerr << "in work No valid color image to display!" << std::endl;
    }
}

k4a::image Work::convert_file2image(const std::string& color_filename) {
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

void Work::getResult(unsigned char* CT, k4a::image& color_image, int CT_width, int CT_height) {
	/*
		we implement it without opencv, because using opencv is too slow to realize real-time display.
		CT length :width * height * 3
		we ensure that color_image's width / height is equal to CT's width / height
		first I should scale CT to color_image.
		Then I should overlay CT on color_image and get the unsigned char* result.
		Pay atteention that result is a member of Work class.we don't need to return it.
		we don't need to new the result every time.Only need to new it once.
		and than display it using opencv.
	*/
    // 获取color_image的宽高
    int width = color_image.get_width_pixels();
    int height = color_image.get_height_pixels();

    // 获取color_image的数据
    uint8_t* color_data = color_image.get_buffer();

    // 分配大小为width * height * 4的result缓冲区，如果尚未分配
    if (result.empty()) {
        result.create(height, width, CV_8UC4);
    }

    // 分配大小为width * height * 3的CT_image缓冲区，用于存储调整尺寸后的CT图像
    unsigned char* CT_resized = new unsigned char[width * height * 3];

    // 实现CT图像的缩放（简单的双线性插值）
    for (int y = 0; y < height; y++) {
        float src_y = (height - 1.0f - y) * (CT_height - 1.0f) / (height - 1.0f);
        int y0 = (int)src_y;
        int y1 = std::min(y0 + 1, CT_height - 1);
        float y_lerp = src_y - y0;

        for (int x = 0; x < width; x++) {
            float src_x = x * (CT_width - 1.0f) / (width - 1.0f);
            int x0 = (int)src_x;
            int x1 = std::min(x0 + 1, CT_width - 1);
            float x_lerp = src_x - x0;

            for (int c = 0; c < 3; c++) {
                float value = (1 - y_lerp) * ((1 - x_lerp) * CT[(y0 * CT_width + x0) * 3 + c] + x_lerp * CT[(y0 * CT_width + x1) * 3 + c]) +
                    y_lerp * ((1 - x_lerp) * CT[(y1 * CT_width + x0) * 3 + c] + x_lerp * CT[(y1 * CT_width + x1) * 3 + c]);
                CT_resized[(y * width + x) * 3 + c] = (unsigned char)value;
            }
        }
    }

    // 计算亮度（灰度值），并进行gamma校正，作为alpha通道
    float gamma = 0.4f;
    unsigned char* alpha_channel = new unsigned char[width * height];

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = (y * width + x);
            unsigned char r = CT_resized[idx * 3 + 2];
            unsigned char g = CT_resized[idx * 3 + 1];
            unsigned char b = CT_resized[idx * 3 + 0];
            // 计算灰度值
            unsigned char gray = (unsigned char)(0.299f * r + 0.587f * g + 0.114f * b);
            // 归一化到[0,1]并进行gamma校正
            float alpha = powf(gray / 255.0f, gamma);
            // 转换回[0,255]
            alpha_channel[idx] = (unsigned char)(alpha * 255.0f);
        }
    }

    // 开始进行alpha混合
    uint8_t* result_data = result.data;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = (y * width + x);

            // 前景色（CT图像）
            float f_b = CT_resized[idx * 3 + 0] / 255.0f;
            float f_g = CT_resized[idx * 3 + 1] / 255.0f;
            float f_r = CT_resized[idx * 3 + 2] / 255.0f;
            float f_a = alpha_channel[idx] / 255.0f;

            // 背景色（color_image）
            float b_b = color_data[idx * 4 + 0] / 255.0f;
            float b_g = color_data[idx * 4 + 1] / 255.0f;
            float b_r = color_data[idx * 4 + 2] / 255.0f;
            // 注意：color_image可能有alpha通道，但这里我们不需要

            // 进行alpha混合
            float out_b = f_b * f_a + b_b * (1.0f - f_a);
            float out_g = f_g * f_a + b_g * (1.0f - f_a);
            float out_r = f_r * f_a + b_r * (1.0f - f_a);

            // 结果存入result
            result_data[idx * 4 + 0] = (unsigned char)(out_b * 255.0f);
            result_data[idx * 4 + 1] = (unsigned char)(out_g * 255.0f);
            result_data[idx * 4 + 2] = (unsigned char)(out_r * 255.0f);
            result_data[idx * 4 + 3] = 255; // alpha通道设为不透明
        }
    }

    // 释放临时内存
    delete[] CT_resized;
    delete[] alpha_channel;
	return;
}

// 定义评估拟合质量的函数 by hy
double Work::evaluate_fit_quality(const std::vector<cv::Point>& contour, const cv::RotatedRect& ellipse) {
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

int Work::get_body_location(k4a::image& color_image)
{
    /*
     by hy
    */
    using namespace std;
    using namespace cv;
    // 创建一个向量来存储椭圆的中心点
    vector<pair<int, int>> centers;
    int width = color_image.get_width_pixels();
    int height = color_image.get_height_pixels();
    uint8_t* buffer = color_image.get_buffer();
    // 读取图像
    // 创建一个包含图像数据的 Mat（假设图像格式为 BGRA）
    cv::Mat img_bgra(height, width, CV_8UC4, buffer);

    // 如果需要将图像从 BGRA 转换为 BGR
    cv::Mat img;
    cv::cvtColor(img_bgra, img, cv::COLOR_BGRA2BGR);
    if (img.empty()) {
        //cout << "无法打开或找到图像" << endl;
        return 0;
    }

    // 转换为灰度图像
    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);

    // 高斯模糊（减少噪声）
    Mat blurred;
    GaussianBlur(gray, blurred, Size(5, 5), 0);

    // 自适应阈值分割
    Mat adaptive_thresh;
    adaptiveThreshold(blurred, adaptive_thresh, 255,
        ADAPTIVE_THRESH_GAUSSIAN_C,
        THRESH_BINARY_INV, 11, 2);

    // 形态学操作
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    Mat morph = adaptive_thresh.clone();
    // 形态学开运算（消除小噪声）
    // morphologyEx(adaptive_thresh, morph, MORPH_OPEN, kernel);

    // 形态学闭运算（填充内部空洞）
    morphologyEx(morph, morph, MORPH_CLOSE, kernel);

    // 寻找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(morph, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 绘制轮廓
    Mat contour_img = img.clone();
    drawContours(contour_img, contours, -1, Scalar(255, 0, 0), 2);

    // 设置长轴长度的最大值
    double max_axis = 120.0;  // 根据您的需求设置
    double max_e = 0.85;
    double fit_quality_threshold = 0.002;

    // 拟合椭圆并绘制
    Mat ellipse_img = img.clone();
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = contourArea(contours[i]);
        if (area > 500 && contours[i].size() >= 5) {
            try {
                RotatedRect ellipseRect = fitEllipse(contours[i]);
                Point2f center = ellipseRect.center;
                Size2f axes = ellipseRect.size;
                float angle = ellipseRect.angle;

                double axis1 = axes.width;
                double axis2 = axes.height;
                if (axis1 < axis2) {
                    swap(axis1, axis2);
                }
                double e = sqrt(1.0 - pow(axis2 / axis1, 2));
                //cout << "离心率: " << e << "  长轴长度: " << axis1 << endl;

                // 检查轴长度是否为正
                if (axes.width > 0 && axes.height > 0) {
                    // 应用长轴长度的最大值限制
                    double mean_distance = evaluate_fit_quality(contours[i], ellipseRect);
                    //cout << "拟合质量: " << mean_distance << endl;
                    if (axis1 <= max_axis && e < max_e && mean_distance <= fit_quality_threshold) {
                        //cout << "检测到有效椭圆，中心点: (" << center.x << ", " << center.y << ")" << endl;
                        ellipse(ellipse_img, ellipseRect, Scalar(0, 255, 0), 2);

                        // 将中心点存储到向量中
                        centers.push_back(make_pair(static_cast<int>(center.x), static_cast<int>(center.y)));
                    }
                }
            }
            catch (cv::Exception& e) {
                //cout << "拟合椭圆时出错: " << e.what() << endl;
            }
        }
    }

    // 对中心点进行排序（按照x坐标升序排序，如果需要按照其他方式排序，可以修改排序条件）
    sort(centers.begin(), centers.end(), [](const pair<int, int>& a, const pair<int, int>& b) {
        return a.first < b.first;
        });


    // 输出排序后的中心点
    cout << "排序后的椭圆中心点：" << endl;
    for (const auto& center : centers) {
        cout << "(" << center.first << ", " << center.second << ")" << endl;
    }

	bodylocation_list = centers;

    // 显示结果图像
    /*Mat display_img;
    resize(ellipse_img, display_img, Size(), 0.5, 0.5, INTER_AREA);
    namedWindow("检测到的椭圆", WINDOW_NORMAL);
    imshow("检测到的椭圆", display_img);
    waitKey(0);
    destroyAllWindows();*/
    if (centers.size() != 3) return 0;

    return 1;
}