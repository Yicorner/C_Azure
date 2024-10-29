// Description: get sample from kinect device
#include "getSample.hpp"
#include "processImage.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <conio.h>  // Windows�����ڷ�����ʽ��ȡ��������
#include <thread>
// ���徲̬��Ա����
//std::mutex GetSample::color_mtx;
//std::mutex GetSample::depth_mtx;

void GetSample::get_sample(k4a::device& device, k4a_device_configuration_t& config, int image_id) {
    /*
    get image sample with native kinect sdk method*/
    k4a::capture capture;
    std::cout << "in get_sample ����ͼ����..." << std::endl;
    if (device.get_capture(&capture, std::chrono::milliseconds(5000)))
    {
        // �Ӳ���������л�ȡRGBͼ��
        color_mtx.lock();
        color_image = capture.get_color_image();
        color_mtx.unlock();
        if (color_image)
        {
            std::cout << "Successfully captured a color image!" << std::endl;

            // ��ȡͼ������
            uint8_t* buffer = color_image.get_buffer();
            int width = color_image.get_width_pixels();
            int height = color_image.get_height_pixels();
            size_t size = color_image.get_size();
            printf("Image width: %4d, height: %4d, size: %zu\n", width, height, size);
            // ����ͼ��Ϊ�ļ����籣��ΪԭʼRGB���ݣ�
            std::string filename = "color_image_" + std::to_string(image_id) + ".raw";
            std::ofstream file("captures/" + filename, std::ios::out | std::ios::binary);
            file.write(reinterpret_cast<char*>(buffer), size);
            file.close();

        }
        else
        {
            std::cout << "Failed to capture a color image." << std::endl;
        }
        // �Ӳ���������л�ȡDepthͼ��
		depth_mtx.lock();
        depth_image = capture.get_depth_image();
		depth_mtx.unlock();

        if (depth_image)
        {
            std::cout << "Successfully captured a depth image!" << std::endl;

            // Get depth image data
            uint16_t* depth_buffer = reinterpret_cast<uint16_t*>(depth_image.get_buffer());
            int depth_width = depth_image.get_width_pixels();
            int depth_height = depth_image.get_height_pixels();
            size_t depth_size = depth_image.get_size();
            printf("Depth Image width: %4d, height: %4d, size: %zu\n", depth_width, depth_height, depth_size);

            // Save the depth image as a raw file
            std::string filename = "depth_image_" + std::to_string(image_id) + ".raw";
            std::ofstream depth_file("captures/" + filename, std::ios::out | std::ios::binary);
            depth_file.write(reinterpret_cast<char*>(depth_buffer), depth_size);
            depth_file.close();
        }
        else
        {
            std::cout << "Failed to capture a depth image." << std::endl;
        }
    }
}
void GetSample::display_current_image() {
    if (color_image) {
        // ��ȡͼ��Ļ�����ָ��
        uint8_t* buffer = color_image.get_buffer();

        // ��ȡͼ��Ŀ�ȡ��߶ȺͲ�����ÿ�е��ֽ�����
        int width = color_image.get_width_pixels();
        int height = color_image.get_height_pixels();
        int stride = color_image.get_stride_bytes();

        // �� Kinect ��ͼ��ת��Ϊ OpenCV ��ʽ�� cv::Mat
        // ���� color_image �� RGB ͼ��ÿ��������3���ֽڣ�
        cv::Mat image(height, width, CV_8UC4, buffer, stride);  // ����Ϊ BGRA/RGBA

        // ת��Ϊ BGR ��ʽ��OpenCV ʹ�� BGR ˳��
        cv::Mat image_bgr;
        cv::cvtColor(image, image_bgr, cv::COLOR_BGRA2BGR);

        // ʹ�� OpenCV ��ʾͼ��
        cv::imshow("Color Image in GetSample", image_bgr);

        // �ȴ����������Թرմ���
        cv::waitKey(30);  // �ȴ� 1 ����, �Ա㴰��ˢ��
    }
    else {
        std::cerr << "in getsample No valid color image to display!" << std::endl;
    }
}
void GetSample::get_sample_start(k4a::device& device, k4a_device_configuration_t& config) {
    /*
    the initiation before get_sample, start_cameras. but we can't start camera twice
    */
    try
    {
        //get_sample();
        std::cout << "�������������� 'G' ������ͼ�񣬰� 'Q' ���˳�����" << std::endl;
        bool running = true;
        while (running)
        {
            if (_kbhit())
            {
                // ��ȡ���µļ�
                char ch = _getch();

                if (ch == 'G' || ch == 'g')
                {
                    count++;
                    if (count % 2 == 1) {
                        image_id = count / 2 + 1;
                        get_sample(device, config, image_id);
                        process_color_image(image_id);
                        process_depth_image(image_id);
                    }
                    else {
                        std::cout << "����ͣ" << std::endl;
                    }
                }
                else if (ch == 'Q' || ch == 'q')
                {
                    std::cout << "�˳�����" << std::endl;
                    running = false;
                }
                else
                {
                    std::cout << "δ֪�İ�����" << ch << std::endl;
                    std::cout << "�� 'G' ������ͼ�񣬰� 'Q' ���˳�����" << std::endl;
                }
            }
            // ����һ��ʱ�䣬����ռ�ù���CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            std::cout << "get_sample_start ��������" << std::endl;
			std::cout << "start display static image in getsample" << std::endl;
            display_current_image();
        }
        // ��ȡcolorspace����
        //get_cords(x, y, image_id);
    }
    catch (const k4a::error& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}