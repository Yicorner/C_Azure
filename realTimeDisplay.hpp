#ifndef REAL_TIME_DISPLAY_HPP
#define REAL_TIME_DISPLAY_HPP

// 包含必要的头文件
#include <k4a/k4a.hpp>
#include <atomic>
// 函数声明
class RealTimeDisplay
{
public:
	std::atomic<bool> running;

	RealTimeDisplay() :running(true) {} // 构造函数
	~RealTimeDisplay() {}// 析构函数
	void realTimeDisplay(k4a::device& device);
	void display_image(const k4a::image& k4a_image, const std::string& window_name);
};
#endif // PROCESS_IMAGE_HPP