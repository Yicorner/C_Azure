#ifndef WORK_HPP
#define WORK_HPP
// 包含必要的头文件
#include "getSample.hpp"
#include <opencv2/opencv.hpp>
#include <atomic>
#include "AllEnum.hpp"
#include "GetCords.hpp"
#include "MultiTimer.hpp"
#include "RendererGUI.h"
#include "constants.hpp"

class Get3DCords;
class GetSample;
// 函数声明
// 这个类用于从类GetSample中获取数据并进行处理，得到最终的AR图
class Work
{
public:
	RendererGUI vr;
	k4a::image color_image; // 声明一个颜色图像，用于在类work中获取数据
	k4a::image depth_image; // 声明一个深度图像，用于在类work中获取数据
	std::vector<std::pair<int, int>> bodylocation_list; // 保存人体位置的像素列表
	std::vector<std::vector<float>> body3Dlocation_list; // 保存人体位置的空间位置列表mm
	
	Get3DCords get3dcords; // 用于获取空间位置
	MultiTimer& timer = MultiTimer::getInstance();
	static int frame_id;
	static int save_file_id;
	cv::Mat result; // 保存最终的AR图
	k4a::device* device; // 引用k4a::device对象
	k4a_device_configuration_t* config; // 引用k4a_device_configuration_t对象
	std::atomic<State> state; // 用于记录当前的状态
	BodyLocation bodylocation; // 0代表未识别到，1代表头部，2代表胸部，3代表肚子
	Work(k4a::device& device, k4a_device_configuration_t& config) :
		bodylocation(Constants::body_location), state(Constants::state), device(&device),
		config(&config), get3dcords(device, config), vr(1440, 810, "Volume-Renderer")
	{
		vr.setShaderAndData();
	}; // 构造函数

	~Work() {}// 析构函数
	void run(GetSample& sample); // 开始构建最终的AR图
	int get_body_location(k4a::image& color_image) ;// 获取人体位置
	void getResult(unsigned char* CT, k4a::image& color_image, int CT_width, int CT_height); // 得到最终的AR图
	int resort_3D_bodylocation_list(std::vector<std::vector<float>>& body3Dlocation_list); // 对人体位置列表进行排序
	void run_multi_thread(GetSample& sample);
private:
	k4a::image color_image2; // 声明一个颜色图像，用于在类work中获取数据
	k4a::image depth_image2; // 声明一个深度图像，用于在类work中获取数据
	std::vector<std::pair<int, int>> bodylocation_list2; // 保存人体位置的像素列表
	std::vector<std::vector<float>> body3Dlocation_list2; // 保存人体位置的空间位置列表mm
	bool ready2 = false;

	k4a::image color_image3; // 声明一个颜色图像，用于在类work中获取数据
	k4a::image depth_image3; // 声明一个深度图像，用于在类work中获取数据
	std::vector<std::pair<int, int>> bodylocation_list3; // 保存人体位置的像素列表
	std::vector<std::vector<float>> body3Dlocation_list3; // 保存人体位置的空间位置列表mm
	bool ready3 = false;

	std::mutex color_mtx2;
	std::mutex depth_mtx2;
	std::mutex bodylocation_mtx2;

	std::mutex color_mtx3;
	std::mutex body3Dlocation_mtx3;
	
	std::mutex mtx2;
	std::mutex mtx3;
	std::condition_variable cv2;
	std::condition_variable cv3;
	void loop_get_body_location(); // 循环获取人体位置
	void loop_get_3D_body_location(); // 循环获取人体位置
};
#endif // PROCESS_IMAGE_HPP