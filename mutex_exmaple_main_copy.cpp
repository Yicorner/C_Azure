#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <mutex>

// ȫ�ֱ���
std::mutex img_mutex; // �����������ڱ�����������
cv::Mat color_image_mat;
cv::Mat depth_image_mat;
bool stop_display = false; // ���ڿ�����ʾ�̵߳��˳�

void display_images()
{
    while (!stop_display)
    {
        std::lock_guard<std::mutex> lock(img_mutex); // ȷ��ͼ�����ݷ���ʱ���̰߳�ȫ��
        if (!color_image_mat.empty()) {
            cv::imshow("RGBA Image", color_image_mat);
        }
        if (!depth_image_mat.empty()) {
            cv::imshow("Depth Image", depth_image_mat);
        }
        std::cout << "display_images ��������" << std::endl;
        // �ȴ� 30ms�����м����������˳�
        if (cv::waitKey(30) >= 0) {
            stop_display = true;
        }
    }
}

void process_images(k4a::capture& capture)
{
    k4a::image color_image = capture.get_color_image();
    k4a::image depth_image = capture.get_depth_image();

    if (color_image) {
        int width = color_image.get_width_pixels();
        int height = color_image.get_height_pixels();
        const uint8_t* buffer = color_image.get_buffer();
        std::lock_guard<std::mutex> lock(img_mutex); // ȷ��ͼ�����ݷ���ʱ���̰߳�ȫ��
        color_image_mat = cv::Mat(height, width, CV_8UC4, (void*)buffer).clone(); // ����ͼ��
        std::cout << "color_image_mat" << std::endl;
    }

    if (depth_image) {
        int width = depth_image.get_width_pixels();
        int height = depth_image.get_height_pixels();
        const uint16_t* buffer = reinterpret_cast<const uint16_t*>(depth_image.get_buffer());
        std::lock_guard<std::mutex> lock(img_mutex); // ȷ��ͼ�����ݷ���ʱ���̰߳�ȫ��
        cv::Mat depth_image_raw = cv::Mat(height, width, CV_16U, (void*)buffer).clone();

        // ת�����ͼ��Ϊ 8 λ�Ҷ�ͼ������ʾ
        depth_image_raw.convertTo(depth_image_mat, CV_8U, 255.0 / 4000.0); // ����������Ϊ 4000mm
        std::cout << "depth_image_mat" << std::endl;

    }
}

int main()
{
    // ��Kinect�豸
    k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);

    // �����豸��������ȺͲ�ɫģʽ
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;                 // Ensure synchronized captures
    config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    device.start_cameras(&config);

    // ����ͼ����ʾ�߳�
    std::thread display_thread(display_images);

    k4a::capture capture;
    std::cout << "Capturing images..." << std::endl;

    while (!stop_display) // ʵʱ����ͼ��
    {
        if (device.get_capture(&capture, std::chrono::milliseconds(5000))) {
            process_images(capture);
        }
        // ����������˳�
        if (cv::waitKey(30) >= 0) {
            break;
        }
    }
    // �ȴ���ʾ�߳��˳�
    display_thread.join();

    // ֹͣ������ر��豸
    device.stop_cameras();
    device.close();

    return 0;
}