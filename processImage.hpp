#ifndef PROCESS_IMAGE_HPP
#define PROCESS_IMAGE_HPP

// ������Ҫ��ͷ�ļ�
#include <string>
// ��������
void process_depth_image(int image_id);
void process_color_image(int image_id);
void process_depth_image_with_filename(std::string filename, std::string input_file);

#endif // PROCESS_IMAGE_HPP
