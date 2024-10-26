#ifndef REAL_TIME_DISPLAY_HPP
#define REAL_TIME_DISPLAY_HPP

// ������Ҫ��ͷ�ļ�
#include <k4a/k4a.hpp>
#include <atomic>
// ��������
class RealTimeDisplay
{
public:
	std::atomic<bool> running;

	RealTimeDisplay() :running(true) {} // ���캯��
	~RealTimeDisplay() {}// ��������
	void realTimeDisplay(k4a::device& device);
	void display_image(const k4a::image& k4a_image, const std::string& window_name);
};
#endif // PROCESS_IMAGE_HPP