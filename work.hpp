#ifndef WORK_HPP
#define WORK_HPP
// 包含必要的头文件
#include "getSample.hpp"
#include <opencv2/opencv.hpp>
#include <atomic>
#include "AllEnum.hpp"
class GetSample;
// 函数声明
// 这个类用于从类GetSample中获取数据并进行处理，得到最终的AR图
class Work
{
public:
	int save_file_id;
	cv::Mat result; // 保存最终的AR图
	k4a::device* device; // 引用k4a::device对象
	k4a_device_configuration_t* config = NULL; // 引用k4a_device_configuration_t对象
	std::atomic<State> state; // 用于记录当前的状态
	BodyLocation bodylocation; // 0代表未识别到，1代表头部，2代表胸部，3代表肚子
	std::vector<std::pair<int, int>> bodylocation_list; // 保存人体位置的像素列表
	std::vector<std::vector<float>> body3Dlocation_list; // 保存人体位置的空间位置列表mm
	Work(k4a::device& device, k4a_device_configuration_t& config) :
		bodylocation(BodyLocation::NOT_DETECTED), state(State::DYNAMIC), device(&device), config(&config), save_file_id(0){}; // 构造函数
	~Work() {}// 析构函数
	void run(GetSample& sample); // 开始构建最终的AR图
	void display_image_from_k4aimage(k4a::image& color_image); // 从GetSample中获取数据并显示
	void display_image_from_filename(const std::string& color_filename, const std::string& depth_filename); // 从文件中获取数据并显示
	int get_body_location(k4a::image& color_image) ;// 获取人体位置
	void getResult(unsigned char* CT, k4a::image& color_image, int CT_width, int CT_height); // 得到最终的AR图
	k4a::image convert_file2image(const std::string& color_filename); // this is for debug
	int resort_3D_bodylocation_list(); // 对人体位置列表进行排序
	void save_k4a_color_image_to_png(k4a::image& color_image, const std::string& filename); // 保存k4a::image对象
	void save_mat_image(cv::Mat& image, const std::string& filename); // 保存cv::Mat对象
	void save_depth_image(k4a::image& depth_image, const std::string& filename_no_ext);
	void save_body3Dlocation_list_in_txt_file(const std::string& filename, const std::vector<std::vector<float>>& body3Dlocation_list, const std::vector<std::pair<int,int>> bodylocation_list); // 保存人体位置的空间位置列表mm
	float distance(std::vector<float>& p1, std::vector<float>& p2); // 计算两个点之间的距离
	double evaluate_fit_quality(const std::vector<cv::Point>& contour, const cv::RotatedRect& ellipse);// 定义评估拟合质量的函数
};
#endif // PROCESS_IMAGE_HPP