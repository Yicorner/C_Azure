#ifndef GET_SAMPLE_HPP
#define GET_SAMPLE_HPP

// ������Ҫ��ͷ�ļ�
#include <k4a/k4a.hpp>
#include <atomic>
// ��������
class GetSample
{
public:
	std::atomic<int> count;

	GetSample() :count(0) {} // ���캯��
	~GetSample() {}// ��������
	void get_sample(k4a::device& device, k4a_device_configuration_t& config, int image_id);
	void get_sample_start(k4a::device& device, k4a_device_configuration_t& config);
};
#endif // PROCESS_IMAGE_HPP