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
	void realTimeDisplay(k4a::device& device); // ʵʱ��ʾͼ��
	void display_image(k4a::image& k4a_image, std::string window_name); // ��ʾͼ��
};
#endif // PROCESS_IMAGE_HPP