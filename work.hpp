#ifndef WORK_HPP
#define WORK_HPP
// 包含必要的头文件
#include "getSample.hpp"
#include <opencv2/opencv.hpp>
#include <atomic>
#include "AllEnum.hpp"
#include "GetCords.hpp"
#include "MultiTimer.hpp"

class Get3DCords;
class GetSample;
// 函数声明
// 这个类用于从类GetSample中获取数据并进行处理，得到最终的AR图
class Work
{
public:
	Get3DCords get3dcords; // 用于获取空间位置
	MultiTimer& timer = MultiTimer::getInstance();
	static int frame_id;
	static int save_file_id;
	cv::Mat result; // 保存最终的AR图
	k4a::device* device; // 引用k4a::device对象
	k4a_device_configuration_t* config; // 引用k4a_device_configuration_t对象
	std::atomic<State> state; // 用于记录当前的状态
	BodyLocation bodylocation; // 0代表未识别到，1代表头部，2代表胸部，3代表肚子
	std::vector<std::pair<int, int>> bodylocation_list; // 保存人体位置的像素列表
	std::vector<std::vector<float>> body3Dlocation_list; // 保存人体位置的空间位置列表mm
	Work(k4a::device& device, k4a_device_configuration_t& config) :
		bodylocation(BodyLocation::NOT_DETECTED), state(State::DYNAMIC), device(&device),
		config(&config), get3dcords(device, config) {}; // 构造函数

	~Work() {}// 析构函数
	void run(GetSample& sample); // 开始构建最终的AR图
	int get_body_location(k4a::image& color_image) ;// 获取人体位置
	void getResult(unsigned char* CT, k4a::image& color_image, int CT_width, int CT_height); // 得到最终的AR图
	int resort_3D_bodylocation_list(); // 对人体位置列表进行排序
};
#endif // PROCESS_IMAGE_HPP