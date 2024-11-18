#ifndef WORK_HPP
#define WORK_HPP
// ������Ҫ��ͷ�ļ�
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
// ��������
// ��������ڴ���GetSample�л�ȡ���ݲ����д����õ����յ�ARͼ
class Work
{
public:

	RendererGUI vr;
	k4a::image color_image; // ����һ����ɫͼ����������work�л�ȡ����
	k4a::image depth_image; // ����һ�����ͼ����������work�л�ȡ����
	std::vector<std::pair<int, int>> bodylocation_list; // ��������λ�õ������б�
	std::vector<std::vector<float>> body3Dlocation_list; // ��������λ�õĿռ�λ���б�mm
	
	Get3DCords get3dcords; // ���ڻ�ȡ�ռ�λ��
	MultiTimer& timer = MultiTimer::getInstance();
	static int frame_id;
	static int save_file_id;

	k4a::device* device; // ����k4a::device����
	k4a_device_configuration_t* config; // ����k4a_device_configuration_t����
	std::atomic<State> state; // ���ڼ�¼��ǰ��״̬
	BodyLocation bodylocation; // 0����δʶ�𵽣�1����ͷ����2�����ز���3�������
	Work(k4a::device& device, k4a_device_configuration_t& config) :
		bodylocation(Constants::body_location), state(Constants::state), device(&device),
		config(&config), get3dcords(device, config), vr(Constants::width, Constants::height, "Volume-Renderer", device, config, false)
	{
		// Allocate the result2 buffer if it hasn't been allocated yet
		if (result2.empty()) {
			result2.create(1080, 1920, CV_8UC3);
		}
		// �����СΪwidth * height * 4��result�������������δ����
		if (result.empty()) {
			result.create(1080, 1920, CV_8UC3);
		}
		// �����СΪwidth * height * 4��result_with_cone�������������δ����
		if (result_with_cone.empty()) {
			result_with_cone.create(1080, 1920, CV_8UC3);
		}
		previous_result_ready = false;
		// ��ʼ��color_image��depth_image
		k4a::capture capture;
		if ((device).get_capture(&capture, std::chrono::milliseconds(5000))) {
			color_image = capture.get_color_image();
			depth_image = capture.get_depth_image();
		}
	}; // ���캯��

	~Work() {}// ��������
	void run(GetSample& sample); // ��ʼ�������յ�ARͼ
	int get_body_location(k4a::image& color_image) ;// ��ȡ����λ��
	void getResult(unsigned char* CT, k4a::image& color_image, int CT_width, int CT_height); // �õ����յ�ARͼ
	void getResult2(const cv::Mat& result_with_cone, const cv::Mat& result);
	int resort_3D_bodylocation_list(std::vector<std::vector<float>>& body3Dlocation_list); // ������λ���б��������
	void run_multi_thread(GetSample& sample);
	void run_temp(GetSample& sample);
	void run_multi_thread2(GetSample& sample);
	void run_multi_thread_with_shader(GetSample& sample);
	void getResultWithMat(unsigned char* CT, cv::Mat& color_image, int CT_width, int CT_height);
	void set_body_location(); // ѭ����ȡ����λ��
	void capture_image();
	void set_vertices();
	void set_color_image_to_core();

private:
	k4a::image color_image2; // ����һ����ɫͼ����������work�л�ȡ����
	k4a::image depth_image2; // ����һ�����ͼ����������work�л�ȡ����
	std::vector<std::pair<int, int>> bodylocation_list2; // ��������λ�õ������б�
	std::vector<std::vector<float>> body3Dlocation_list2; // ��������λ�õĿռ�λ���б�mm
	bool ready2 = false;

	k4a::image color_image3; // ����һ����ɫͼ����������work�л�ȡ����
	k4a::image depth_image3; // ����һ�����ͼ����������work�л�ȡ����
	std::vector<std::pair<int, int>> bodylocation_list3; // ��������λ�õ������б�
	std::vector<std::vector<float>> body3Dlocation_list3; // ��������λ�õĿռ�λ���б�mm
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
	void loop_get_body_location(); // ѭ����ȡ����λ��
	void loop_get_3D_body_location(); // ѭ����ȡ����λ��

	cv::Mat result; // �������յ�ARͼ
	cv::Mat result2; // �������յ�ARͼ
	cv::Mat result_with_cone; // �������յ�ARͼ

	bool previous_result_ready;
	cv::Mat previous_result_with_cone;
	cv::Mat previous_result;


};
#endif // PROCESS_IMAGE_HPP