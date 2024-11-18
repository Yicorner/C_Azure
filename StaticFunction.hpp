#ifndef STATICFUNCTION_HPP
#define STATICFUNCTION_HPP

// 包含必要的头文件
#include <glad/glad.h>
#include <string>
#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include "MultiTimer.hpp"
#include "AllEnum.hpp"
#include "Eigen/Dense"
#include <glm/glm.hpp>
// 函数声明
class StaticFunction
{
public:
	static void process_depth_image(int image_id);
	static void process_color_image(int image_id);
	static void process_depth_image_with_filename(std::string filename, std::string input_file);
	static void display_image_from_k4aimage(k4a::image& color_image, const std::string& window_name); // 从GetSample中获取数据并显示
	static double evaluate_fit_quality(const std::vector<cv::Point>& contour, const cv::RotatedRect& ellipse);// 定义评估拟合质量的函数
	static k4a::image convert_file2image(const std::string& color_filename); // this is for debug png to k4a::image
	static void save_k4a_color_image_to_png(k4a::image& color_image, const std::string& filename); // 保存k4a::image对象
	static void save_depth_image_in_raw(k4a::image& depth_image, const std::string& filename); // 保存深度图像

	static void save_mat_image(cv::Mat& image, const std::string& filename); // 保存cv::Mat对象
	static void save_body3Dlocation_list_in_txt_file(const std::string& filename, const std::vector<std::vector<float>>& body3Dlocation_list, const std::vector<std::pair<int, int>> bodylocation_list); // 保存人体位置的空间位置列表mm
	static float distance(std::vector<float>& p1, std::vector<float>& p2); // 计算两个点之间的距离
	static void display_image_from_filename(const std::string& color_filename, const std::string& depth_filename); // 从文件中获取数据并显示
	static void removeInvalidLocations(std::vector<std::vector<float>>& body3Dlocation_list, const std::vector<float>& comp); // 移除无效的位置
	static cv::Mat k4aImageToCvMat(const k4a::image& k4aImage);
	static cv::Mat addCone(const k4a::image& k4aImage);

	static float crossProduct2D(const std::vector<float>& A, const std::vector<float>& B, const std::vector<float>& C);
	static bool isSameOrder(const std::vector<float>& chest1, const std::vector<float>& chest2, const std::vector<float>& chest3, const std::vector<float>& a, const std::vector<float>& b, const std::vector<float>& c);
	static MultiTimer& timer;
	static void renderTeapotToFBO(GLuint fbo, unsigned int width, unsigned int height, char* imageData);
	static glm::mat4 get_transform(BodyLocation bodylocation, std::vector<std::vector<float>> body3Dlocation_list);
	static glm::mat4 transform_depthCamera2fakeman(glm::mat3 depthCameraColorCord, glm::mat3 NDCfakemanCord);
	static float cal_vec3dist(glm::vec3 a, glm::vec3 b);
	static std::vector<float>  ConeVertice(const k4a::image& k4aImage);
	static cv::Point2d pixelToNDC(float x, float y);
};
#endif