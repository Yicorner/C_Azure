#ifndef GET_SAMPLE_HPP
#define GET_SAMPLE_HPP

// 包含必要的头文件
#include <k4a/k4a.hpp>
#include <atomic>
// 函数声明
class GetSample
{
public:
	std::atomic<int> count;

	GetSample() :count(0) {} // 构造函数
	~GetSample() {}// 析构函数
	void get_sample(k4a::device& device, k4a_device_configuration_t& config, int image_id);
	void get_sample_start(k4a::device& device, k4a_device_configuration_t& config);
};
#endif // PROCESS_IMAGE_HPP