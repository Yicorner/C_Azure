#ifndef STATICFUNCTION_HPP
#define STATICFUNCTION_HPP

// ������Ҫ��ͷ�ļ�
#include <string>
#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
// ��������
class StaticFunction
{
public:
	static void process_depth_image(int image_id);
	static void process_color_image(int image_id);
	static void process_depth_image_with_filename(std::string filename, std::string input_file);
	static void display_image_from_k4aimage(k4a::image& color_image, const std::string& window_name); // ��GetSample�л�ȡ���ݲ���ʾ
	static double evaluate_fit_quality(const std::vector<cv::Point>& contour, const cv::RotatedRect& ellipse);// ����������������ĺ���
	static k4a::image convert_file2image(const std::string& color_filename); // this is for debug png to k4a::image
	static void save_k4a_color_image_to_png(k4a::image& color_image, const std::string& filename); // ����k4a::image����
	static void save_depth_image_in_raw(k4a::image& depth_image, const std::string& filename); // �������ͼ��

	static void save_mat_image(cv::Mat& image, const std::string& filename); // ����cv::Mat����
	static void save_body3Dlocation_list_in_txt_file(const std::string& filename, const std::vector<std::vector<float>>& body3Dlocation_list, const std::vector<std::pair<int, int>> bodylocation_list); // ��������λ�õĿռ�λ���б�mm
	static float distance(std::vector<float>& p1, std::vector<float>& p2); // ����������֮��ľ���
	static void display_image_from_filename(const std::string& color_filename, const std::string& depth_filename); // ���ļ��л�ȡ���ݲ���ʾ
};
#endif