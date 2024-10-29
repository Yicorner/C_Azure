#ifndef WORK_HPP
#define WORK_HPP
// ������Ҫ��ͷ�ļ�
#include "getSample.hpp"
#include <opencv2/opencv.hpp>
#include <atomic>
#include "AllEnum.hpp"
class GetSample;
// ��������
// ��������ڴ���GetSample�л�ȡ���ݲ����д����õ����յ�ARͼ
class Work
{
public:
	int save_file_id;
	cv::Mat result; // �������յ�ARͼ
	k4a::device* device; // ����k4a::device����
	k4a_device_configuration_t* config = NULL; // ����k4a_device_configuration_t����
	std::atomic<State> state; // ���ڼ�¼��ǰ��״̬
	BodyLocation bodylocation; // 0����δʶ�𵽣�1����ͷ����2�����ز���3�������
	std::vector<std::pair<int, int>> bodylocation_list; // ��������λ�õ������б�
	std::vector<std::vector<float>> body3Dlocation_list; // ��������λ�õĿռ�λ���б�mm
	Work(k4a::device& device, k4a_device_configuration_t& config) :
		bodylocation(BodyLocation::NOT_DETECTED), state(State::DYNAMIC), device(&device), config(&config), save_file_id(0){}; // ���캯��
	~Work() {}// ��������
	void run(GetSample& sample); // ��ʼ�������յ�ARͼ
	void display_image_from_k4aimage(k4a::image& color_image); // ��GetSample�л�ȡ���ݲ���ʾ
	void display_image_from_filename(const std::string& color_filename, const std::string& depth_filename); // ���ļ��л�ȡ���ݲ���ʾ
	int get_body_location(k4a::image& color_image) ;// ��ȡ����λ��
	void getResult(unsigned char* CT, k4a::image& color_image, int CT_width, int CT_height); // �õ����յ�ARͼ
	k4a::image convert_file2image(const std::string& color_filename); // this is for debug
	int resort_3D_bodylocation_list(); // ������λ���б��������
	void save_k4a_color_image_to_png(k4a::image& color_image, const std::string& filename); // ����k4a::image����
	void save_mat_image(cv::Mat& image, const std::string& filename); // ����cv::Mat����
	void save_depth_image(k4a::image& depth_image, const std::string& filename_no_ext);
	void save_body3Dlocation_list_in_txt_file(const std::string& filename, const std::vector<std::vector<float>>& body3Dlocation_list, const std::vector<std::pair<int,int>> bodylocation_list); // ��������λ�õĿռ�λ���б�mm
	float distance(std::vector<float>& p1, std::vector<float>& p2); // ����������֮��ľ���
	double evaluate_fit_quality(const std::vector<cv::Point>& contour, const cv::RotatedRect& ellipse);// ����������������ĺ���
};
#endif // PROCESS_IMAGE_HPP