#ifndef WORK_HPP
#define WORK_HPP
// ������Ҫ��ͷ�ļ�
#include "getSample.hpp"
#include <opencv2/opencv.hpp>
#include <atomic>
#include "AllEnum.hpp"
#include "GetCords.hpp"
#include "MultiTimer.hpp"

class Get3DCords;
class GetSample;
// ��������
// ��������ڴ���GetSample�л�ȡ���ݲ����д����õ����յ�ARͼ
class Work
{
public:
	Get3DCords get3dcords; // ���ڻ�ȡ�ռ�λ��
	MultiTimer& timer = MultiTimer::getInstance();
	static int frame_id;
	static int save_file_id;
	cv::Mat result; // �������յ�ARͼ
	k4a::device* device; // ����k4a::device����
	k4a_device_configuration_t* config; // ����k4a_device_configuration_t����
	std::atomic<State> state; // ���ڼ�¼��ǰ��״̬
	BodyLocation bodylocation; // 0����δʶ�𵽣�1����ͷ����2�����ز���3�������
	std::vector<std::pair<int, int>> bodylocation_list; // ��������λ�õ������б�
	std::vector<std::vector<float>> body3Dlocation_list; // ��������λ�õĿռ�λ���б�mm
	Work(k4a::device& device, k4a_device_configuration_t& config) :
		bodylocation(BodyLocation::NOT_DETECTED), state(State::DYNAMIC), device(&device),
		config(&config), get3dcords(device, config) {}; // ���캯��

	~Work() {}// ��������
	void run(GetSample& sample); // ��ʼ�������յ�ARͼ
	int get_body_location(k4a::image& color_image) ;// ��ȡ����λ��
	void getResult(unsigned char* CT, k4a::image& color_image, int CT_width, int CT_height); // �õ����յ�ARͼ
	int resort_3D_bodylocation_list(); // ������λ���б��������
};
#endif // PROCESS_IMAGE_HPP