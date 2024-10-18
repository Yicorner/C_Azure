#include "processImage.hpp"


void process_depth_image(int image_id) {
    // �������ͼ��ߴ�
    const int depth_width = 640;
    const int depth_height = 576;

    // �����ļ���
    std::string filename = "depth_image_" + std::to_string(image_id);
    std::string input_file = "captures/" + filename + ".raw";

    // ��ԭʼ��������ļ�
    std::ifstream file(input_file, std::ios::binary);
    if (!file) {
        std::cerr << "�޷����ļ���" << input_file << std::endl;
        return;
    }

    // ����Ԥ�ڵ����ݴ�С
    size_t expected_size = depth_width * depth_height * sizeof(uint16_t); // ÿ������2�ֽڣ�uint16_t����
    std::vector<uint16_t> buffer(depth_width * depth_height);

    // ��ȡԭʼ�������
    file.read(reinterpret_cast<char*>(buffer.data()), expected_size);
    size_t read_size = file.gcount();

    // ����ȡ�����ݴ�С�Ƿ�ƥ��
    if (read_size != expected_size) {
        std::cerr << "���棺Ԥ�����ݴ�СΪ " << expected_size << " �ֽڣ���ʵ�ʶ�ȡ�� " << read_size << " �ֽڡ�" << std::endl;
        // ������Ҫ���������ƥ��
    }

    // �ر��ļ�
    file.close();

    // ������ת��Ϊ OpenCV �� Mat ����
    cv::Mat depth_image(depth_height, depth_width, CV_16UC1, buffer.data());

    // �����������Ƿ�ȫ��Ϊ��
    double min_depth, max_depth;
    cv::minMaxLoc(depth_image, &min_depth, &max_depth, nullptr, nullptr, depth_image > 0);

    if ((min_depth == 0 && max_depth == 0) || std::isnan(min_depth) || std::isnan(max_depth)) {
        std::cerr << "������ݽ�������ֵ��" << std::endl;
        return;
    }

    // �����ֵת��Ϊ�������Ա㴦��
    cv::Mat depth_in_meters;
    depth_image.convertTo(depth_in_meters, CV_32F);

    // ��һ�����ֵ������ 1000 ���ضϴ��� 1 ��ֵ
    depth_in_meters = depth_in_meters / 1000.0;
    cv::threshold(depth_in_meters, depth_in_meters, 1.0, 1.0, cv::THRESH_TRUNC);

    // �����ֵ���ŵ� 0 - 255 �ķ�Χ
    cv::Mat normalized_depth = depth_in_meters * 255.0;

    // ת��Ϊ uint8 ����
    normalized_depth.convertTo(normalized_depth, CV_8UC1);

    // ����һ�� RGB ͼ��ÿ��ͨ�����ǹ�һ��������ͼ��
    cv::Mat rgb_image;
    cv::cvtColor(normalized_depth, rgb_image, cv::COLOR_GRAY2BGR);

    // ����ͼ��Ϊ PNG �ļ�
    std::string output_file = "captures/" + filename + ".png";
    if (!cv::imwrite(output_file, rgb_image)) {
        std::cerr << "�޷�����ͼ���ļ���" << output_file << std::endl;
        return;
    }

    std::cout << "���ͼ��ɹ�����Ϊ��" << output_file << std::endl;
}

void process_color_image(int image_id) {
    // ����ͼ��ߴ�
    const int width = 1920;
    const int height = 1080;

    // �����ļ���
    std::string filename = "color_image_" + std::to_string(image_id);
    std::string input_file = "captures/" + filename + ".raw";

    // ��ԭʼ�����ļ�
    std::ifstream file(input_file, std::ios::binary);
    if (!file) {
        std::cerr << "�޷����ļ���" << input_file << std::endl;
        return;
    }

    // ����Ԥ�ڵ����ݴ�С
    size_t expected_size = width * height * 4; // 4 �ֽ�ÿ���أ�RGBA��
    std::vector<unsigned char> buffer(expected_size);

    // ��ȡԭʼ����
    file.read(reinterpret_cast<char*>(buffer.data()), expected_size);
    size_t read_size = file.gcount();

    // ����ȡ�����ݴ�С�Ƿ�ƥ��
    if (read_size != expected_size) {
        std::cerr << "���棺Ԥ�����ݴ�СΪ " << expected_size << "����ʵ�ʶ�ȡ�� " << read_size << " �ֽڡ�" << std::endl;
        // ������Ҫ���������ƥ��
    }

    // ������ת��Ϊ OpenCV �� Mat ����
    cv::Mat image(height, width, CV_8UC4, buffer.data());

    // ���ͼ���ǵ��õģ�����ѡ�����·�ת
    // cv::flip(image, image, 0);

    // ����ͼ��Ϊ PNG �ļ�
    std::string output_file = "captures/" + filename + ".png";
    if (!cv::imwrite(output_file, image)) {
        std::cerr << "�޷�����ͼ���ļ���" << output_file << std::endl;
        return;
    }

    std::cout << "ͼ��ɹ�����Ϊ��" << output_file << std::endl;
}