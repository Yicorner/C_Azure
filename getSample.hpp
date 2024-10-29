#ifndef GET_SAMPLE_HPP
#define GET_SAMPLE_HPP

// 包含必要的头文件
#include <k4a/k4a.hpp>
#include <atomic>
#include <mutex>
// 函数声明
class Work;  // 前向声明 Work 类

class GetSample
{
public:
	std::atomic<int> count; // 用于计数，记录当前按了几下G键
	std::atomic<int> image_id; // 用于记录当前的图像id
	k4a::image color_image; // 声明一个颜色图像，用于在类work中获取数据
	k4a::image depth_image; // 声明一个深度图像，用于在类work中获取数据
	std::mutex color_mtx; // 用于保护color_image的互斥锁，防止work一边读，getsample一边写
	std::mutex depth_mtx; // 用于保护depth_image的互斥锁，防止work一边读，getsample一边写
	friend class Work;
	GetSample() :count(0), image_id(0), color_image(NULL), depth_image(NULL) {} // 构造函数
	~GetSample() {}// 析构函数
	void get_sample(k4a::device& device, k4a_device_configuration_t& config, int image_id);
	void get_sample_start(k4a::device& device, k4a_device_configuration_t& config);
	void display_current_image();
};
#endif // PROCESS_IMAGE_HPP