#ifndef GET_SAMPLE_HPP
#define GET_SAMPLE_HPP

// ������Ҫ��ͷ�ļ�
#include <k4a/k4a.hpp>
#include <atomic>
#include <mutex>
// ��������
class Work;  // ǰ������ Work ��

class GetSample
{
public:
	std::atomic<int> count; // ���ڼ�������¼��ǰ���˼���G��
	std::atomic<int> image_id; // ���ڼ�¼��ǰ��ͼ��id
	k4a::image color_image; // ����һ����ɫͼ����������work�л�ȡ����
	k4a::image depth_image; // ����һ�����ͼ����������work�л�ȡ����
	std::mutex color_mtx; // ���ڱ���color_image�Ļ���������ֹworkһ�߶���getsampleһ��д
	std::mutex depth_mtx; // ���ڱ���depth_image�Ļ���������ֹworkһ�߶���getsampleһ��д
	friend class Work;
	GetSample() :count(0), image_id(0), color_image(NULL), depth_image(NULL) {} // ���캯��
	~GetSample() {}// ��������
	void get_sample(k4a::device& device, k4a_device_configuration_t& config, int image_id);
	void get_sample_start(k4a::device& device, k4a_device_configuration_t& config);
	void display_current_image();
};
#endif // PROCESS_IMAGE_HPP