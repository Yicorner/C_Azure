#include <k4a/k4a.hpp>
#include <conio.h>  // Windows�����ڷ�����ʽ��ȡ��������
#include <chrono>
#include <thread>
#include "processImage.hpp"
#include "GetCords.hpp"
#include <opencv2/highgui.hpp>
#include <filesystem>
#include <iostream>
void get_sample(k4a::device& device, k4a_device_configuration_t& config, int image_id) {
    k4a::capture capture;
	std::cout << "in get_sample ����ͼ����..." << std::endl;
    if (device.get_capture(&capture, std::chrono::milliseconds(5000)))
    {
        // �Ӳ���������л�ȡRGBͼ��
        k4a::image color_image = capture.get_color_image();

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
        k4a::image depth_image = capture.get_depth_image();

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
// //����ȫ���̶߳��󣬻�������Ҫ�ĵط������߳�
//std::thread t;
//void preview_image_opencv() {
//    // ��Ĭ��������豸IDΪ0��
//    cv::VideoCapture cap(0);
//
//    if (!cap.isOpened()) {
//        std::cerr << "�޷��������" << std::endl;
//        return;
//    }
//    else {
//        std::cout << "����Ѵ򿪡�" << std::endl;
//    }
//
//    // ��������ֱ��ʣ���ѡ��
//    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
//    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
//
//    // ��ʼ������
//    const std::string window_name = "Camera Preview";
//
//    cv::Mat frame;
//    while (1)
//    {
//        cap >> frame;
//        if (frame.empty())
//            break;
//        cv::imshow("video", frame);
//        if (cv::waitKey(200) > 0)//����������˳�����ͷ
//            break;
//    }
//    cap.release();
//    cv::destroyAllWindows();//�ر����д�
//}
//void run_preview() {
//    // �ڶ����߳������� preview_image_opencv
//    t = std::thread(preview_image_opencv);
//}
int main()
{

    try
    {
        //���߳�����preview_image_opencv
		//run_preview();
        // Open the device
        k4a::device device = k4a::device::open(0);
        // Configure the device
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.color_resolution = K4A_COLOR_RESOLUTION_1080P;   // 1920x1080
        config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;       // 640x576
        config.camera_fps = K4A_FRAMES_PER_SECOND_5;
        config.synchronized_images_only = true;                 // Ensure synchronized captures
        config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;

        // Start the cameras
        device.start_cameras(&config);
        //get_sample();
        std::cout << "�������������� 'G' ������ͼ�񣬰� 'Q' ���˳�����" << std::endl;



        
        
        bool running = true;
        int image_id = 0;
        while (running)
        {   
            if (_kbhit())
            {
                // ��ȡ���µļ�
                char ch = _getch();

                if (ch == 'G' || ch == 'g')
                {
                    image_id++;
                    get_sample(device, config, image_id);
                    process_color_image(image_id);
                    process_depth_image(image_id);
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
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
			std::cout << "��������" << std::endl;
        }
        // ��ȡcolorspace����
		//get_cords(x, y, image_id);
    }
    catch (const k4a::error& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }



    //// ȷ�����̵߳ȴ����߳̽���
    //if (t.joinable()) {
    //    t.join();
    //}
    return 0;
}
