#include "work.hpp"
#include "RendererGUI.h"
#include "AllEnum.hpp"
#include "stb_image_write.h"
#include "StaticFunction.hpp"
#include "constants.hpp"

#include <omp.h>
#include <iostream>
#include <k4a/k4a.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <thread>
#include <future>
// defined static variant for debug
int Work::frame_id = 0;
int Work::save_file_id = 0;

void Work::run(GetSample& sample)
{
	// inital a object of k4a::capture for capture both color and depth iamge
    k4a::capture capture;
	// this id is used to save the image and txt file to debug
    // 创建窗口（在循环外创建一次）,大小为960*540
    cv::namedWindow(Constants::window_name, cv::WINDOW_AUTOSIZE);
    cv::resizeWindow(Constants::window_name, 960, 540);
    cv::moveWindow(Constants::window_name, 1000, 100);
    while (1) {
        if (state == FROM_FILE) {
            /*
				调试状态，从文件中读取图片，然后进行处理。
            */
            body3Dlocation_list.clear();
            const std::string color_filename = Constants::from_file_color_filename;
            const std::string depth_filename = Constants::from_file_depth_filename;
            // get body3Dlocation_list
            k4a::image color_image = StaticFunction::convert_file2image(color_filename);
			get_body_location(color_image);
			for (int i = 0; i < bodylocation_list.size(); i++) {
                get3dcords.get_cords_with_filename(bodylocation_list[i].first, bodylocation_list[i].second, depth_filename, body3Dlocation_list);
			}
            resort_3D_bodylocation_list(body3Dlocation_list);
            // the color image of color_filename is the abdomen of the patient
            bodylocation = Constants::body_location;
			// start volume rendering
            vr.run_only_render(bodylocation, body3Dlocation_list, Constants::minval, Constants::maxval, Constants::alpha);
            // get pointer which point to the CT volume render result
            vr.run_only_image_content();
            unsigned char* CT = vr.volren.img_data_from_core;
            // blend the color image and the CT volume render result to get the final blend result in variant result
            getResult(CT, color_image, vr.volren.framebuffer_size.x, vr.volren.framebuffer_size.y);
            //show final blend result
            cv::Mat resized_result;
            cv::resize(result, resized_result, cv::Size(960, 540));
            cv::imshow(Constants::window_name, resized_result);
            cv::moveWindow(Constants::window_name, 1000, 100);
            cv::waitKey(30);
        }
        else if (state == DYNAMIC) {
            frame_id++;
            if ((*device).get_capture(&capture, std::chrono::milliseconds(5000))) {
                std::cout << "start frame: " << frame_id <<  " ------------------------------------------------------------------------------------------" << std::endl;
                
                timer.start("Code Segment capture");
                color_image = capture.get_color_image();
                depth_image = capture.get_depth_image();
                timer.stop("Code Segment capture");

                if (color_image && depth_image) {
                    body3Dlocation_list.clear();
                    get_body_location(color_image);

                    for (int i = 0; i < bodylocation_list.size(); i++) {
						// from pixel cordinates in color_image to 3D cordinates in color_image space cordinates
                        get3dcords.get_cords_with_depth_image(bodylocation_list[i].first, bodylocation_list[i].second, depth_image, body3Dlocation_list);
                    }
                    int detect_state = resort_3D_bodylocation_list(body3Dlocation_list);
                    if (detect_state == 1) {
                        save_file_id++;
						std::cout << "get valid image id: " + std::to_string(save_file_id) + "   *****************************************************************************" << std::endl;
						// save body3Dlocation_list in txt file
                        //StaticFunction::save_body3Dlocation_list_in_txt_file(Constants::save_dir + "bodylocation_" + std::to_string(save_file_id) + ".txt", body3Dlocation_list, bodylocation_list);
						// Save color image
                        //StaticFunction::save_k4a_color_image_to_png(color_image, Constants::save_dir + "color_" + std::to_string(save_file_id) + ".png");
						// Save depth image in raw
                        //StaticFunction::save_depth_image_in_raw(depth_image, Constants::save_dir + "depth_" + std::to_string(save_file_id) + ".raw");
						// shift depth raw to png for visualization
                        //StaticFunction::process_depth_image_with_filename(Constants::save_dir + "depth_" + std::to_string(save_file_id) + ".png", Constants::save_dir + "depth_" + std::to_string(save_file_id) + ".raw");
						// start volume rendering
                        bodylocation = Constants::body_location; //qbh
                        //bodylocation = BodyLocation::CHEST;
                        //body3Dlocation_list[0] = { 11.4616f, 3.58519f, 134.144f };
                        //body3Dlocation_list[1] = { -24.4033f, -35.7827f, 150.218f };
                        //body3Dlocation_list[2] = { -56.8212f, 25.2224f, 142.874f };
                        
                        vr.run_only_render(bodylocation, body3Dlocation_list, Constants::minval, Constants::maxval, Constants::alpha);
						vr.run_only_image_content();
                        // get pointer which point to the CT volume render result
						unsigned char* CT = vr.volren.img_data_from_core;
						// blend the color image and the CT volume render result to get the final blend result in variant result
						getResult(CT, color_image, vr.volren.framebuffer_size.x, vr.volren.framebuffer_size.y);
						//show final blend result
						cv::Mat resized_result;
                        cv::resize(result, resized_result, cv::Size(960, 540));
						cv::imshow(Constants::window_name, resized_result);
                        cv::moveWindow(Constants::window_name, 1000, 100);
                        // Dynamically update the window title
                        std::string dynamic_title = Constants::window_name + " " + std::to_string(save_file_id);
                        cv::setWindowTitle(Constants::window_name, dynamic_title);  // 更新标题
						cv::waitKey(30);
						timer.printStatistics();
                        // save final blend result
						//StaticFunction::save_mat_image(result, Constants::save_dir + "result_" + std::to_string(save_file_id) + ".png");

						/*
						* the code below for debug to verify the correctness of CT
						* stbi_flip_vertically_on_write(1);
						* std::string fn = "D:/data/project/VisualStudio/C_Azure/rendered-image/2.png";
						* int stride = (vr.volren.framebuffer_size.x % 4) + (vr.volren.framebuffer_size.x * 3);
						* bool status = stbi_write_png(fn.c_str(), vr.volren.framebuffer_size.x, vr.volren.framebuffer_size.y, 3, CT, stride);
						*/
						// save CT volume render image
                        //stbi_flip_vertically_on_write(1);
                        //std::string fn = Constants::save_dir + "CT_" + std::to_string(save_file_id) + ".png";
                        //int stride = (vr.volren.framebuffer_size.x % 4) + (vr.volren.framebuffer_size.x * 3);
                        //bool status = stbi_write_png(fn.c_str(), vr.volren.framebuffer_size.x, vr.volren.framebuffer_size.y, 3, CT, stride);
                    }
                    else {
                        std::cout << "未识别到三个点，不进行处理" << std::endl;
                    }
                }
            }
        }
        else {
            assert(0 && "other state remain to implement");
        }
    }
}

void Work::run_temp(GetSample& sample) {
    // inital a object of k4a::capture for capture both color and depth iamge
    k4a::capture capture;
    // this id is used to save the image and txt file to debug
    // 创建窗口（在循环外创建一次）,大小为960*540
    cv::namedWindow(Constants::window_name, cv::WINDOW_AUTOSIZE);
    cv::resizeWindow(Constants::window_name, 1440, 810);
    cv::moveWindow(Constants::window_name, 500, 100);
    while (1) {
       if (state == DYNAMIC) {
            frame_id++;
            if ((*device).get_capture(&capture, std::chrono::milliseconds(5000))) {
                std::cout << "start frame: " << frame_id << " ------------------------------------------------------------------------------------------" << std::endl;

                timer.start("Code Segment capture");
                color_image = capture.get_color_image();
                depth_image = capture.get_depth_image();
                timer.stop("Code Segment capture");

                if (color_image && depth_image) {
                    //body3Dlocation_list.clear();
                    //get_body_location(color_image);

                    //for (int i = 0; i < bodylocation_list.size(); i++) {
                    //    // from pixel cordinates in color_image to 3D cordinates in color_image space cordinates
                    //    get3dcords.get_cords_with_depth_image(bodylocation_list[i].first, bodylocation_list[i].second, depth_image, body3Dlocation_list);
                    //}
                    //int detect_state = resort_3D_bodylocation_list(body3Dlocation_list);
					int detect_state = 1;
                    if (detect_state == 1) {
                        save_file_id++;
                        std::cout << "get valid image id: " + std::to_string(save_file_id) + "   *****************************************************************************" << std::endl;
                        // save body3Dlocation_list in txt file
                        //StaticFunction::save_body3Dlocation_list_in_txt_file(Constants::save_dir + "bodylocation_" + std::to_string(save_file_id) + ".txt", body3Dlocation_list, bodylocation_list);
                        // Save color image
                        //StaticFunction::save_k4a_color_image_to_png(color_image, Constants::save_dir + "color_" + std::to_string(save_file_id) + ".png");
                        // Save depth image in raw
                        //StaticFunction::save_depth_image_in_raw(depth_image, Constants::save_dir + "depth_" + std::to_string(save_file_id) + ".raw");
                        // shift depth raw to png for visualization
                        //StaticFunction::process_depth_image_with_filename(Constants::save_dir + "depth_" + std::to_string(save_file_id) + ".png", Constants::save_dir + "depth_" + std::to_string(save_file_id) + ".raw");
                        // start volume rendering
                        //bodylocation = Constants::body_location; //qbh
                        bodylocation = BodyLocation::CHEST;
                        body3Dlocation_list.resize(3);
                        body3Dlocation_list[0] = { 11.4616f, 3.58519f, 134.144f };
                        body3Dlocation_list[1] = { -24.4033f, -35.7827f, 150.218f };
                        body3Dlocation_list[2] = { -56.8212f, 25.2224f, 142.874f };

                        vr.run_only_render(bodylocation, body3Dlocation_list, Constants::minval, Constants::maxval, Constants::alpha);
                        vr.run_only_image_content();
                        // get pointer which point to the CT volume render result
                        unsigned char* CT = vr.volren.img_data_from_core;
                        // blend the color image and the CT volume render result to get the final blend result in variant result
                        getResult(CT, color_image, vr.volren.framebuffer_size.x, vr.volren.framebuffer_size.y);
                        //show final blend result
                        cv::Mat resized_result;
                        cv::resize(result, resized_result, cv::Size(1440, 810));
                        cv::imshow(Constants::window_name, resized_result);
                        cv::moveWindow(Constants::window_name, 500, 100);
                        // Dynamically update the window title
                        //std::string dynamic_title = Constants::window_name + " " + std::to_string(save_file_id);
                        //cv::setWindowTitle(Constants::window_name, dynamic_title);  // 更新标题
                        cv::waitKey(30);
                        timer.printStatistics();
                        // save final blend result
                        //StaticFunction::save_mat_image(result, Constants::save_dir + "result_" + std::to_string(save_file_id) + ".png");

                        /*
                        * the code below for debug to verify the correctness of CT
                        * stbi_flip_vertically_on_write(1);
                        * std::string fn = "D:/data/project/VisualStudio/C_Azure/rendered-image/2.png";
                        * int stride = (vr.volren.framebuffer_size.x % 4) + (vr.volren.framebuffer_size.x * 3);
                        * bool status = stbi_write_png(fn.c_str(), vr.volren.framebuffer_size.x, vr.volren.framebuffer_size.y, 3, CT, stride);
                        */
                        // save CT volume render image
                        //stbi_flip_vertically_on_write(1);
                        //std::string fn = Constants::save_dir + "CT_" + std::to_string(save_file_id) + ".png";
                        //int stride = (vr.volren.framebuffer_size.x % 4) + (vr.volren.framebuffer_size.x * 3);
                        //bool status = stbi_write_png(fn.c_str(), vr.volren.framebuffer_size.x, vr.volren.framebuffer_size.y, 3, CT, stride);
                    }
                    else {
                        std::cout << "未识别到三个点，不进行处理" << std::endl;
                    }
                }
            }
        }
        else {
            assert(0 && "other state remain to implement");
        }
    }
}
void Work::run_multi_thread(GetSample& sample) {
    if (state == DYNAMIC) {
        std::thread t1(&Work::loop_get_body_location, this);
        std::thread t2(&Work::loop_get_3D_body_location, this);
        // 创建窗口（在循环外创建一次）,大小为960*540
        cv::namedWindow(Constants::window_name, cv::WINDOW_AUTOSIZE);
        cv::resizeWindow(Constants::window_name, 960, 540);
        cv::moveWindow(Constants::window_name, 1000, 100);
        while (1) {
            ready3 = false;
            std::unique_lock<std::mutex> lock(mtx3);
            while (!ready3)
                cv3.wait(lock); // temp
            save_file_id++;
            std::cout << "get valid image id: " + std::to_string(save_file_id) + "   *****************************************************************************" << std::endl;
            // save body3Dlocation_list in txt file
            // Save color image
            //StaticFunction::save_k4a_color_image_to_png(color_image, Constants::save_dir + "color_" + std::to_string(save_file_id) + ".png");
            // Save depth image in raw
            //StaticFunction::save_depth_image_in_raw(depth_image, Constants::save_dir + "depth_" + std::to_string(save_file_id) + ".raw");
            // shift depth raw to png for visualization
            //StaticFunction::process_depth_image_with_filename(Constants::save_dir + "depth_" + std::to_string(save_file_id) + ".png", Constants::save_dir + "depth_" + std::to_string(save_file_id) + ".raw");
            // start volume rendering
            bodylocation = Constants::body_location; // gai

            body3Dlocation_mtx3.lock();
            vr.run_only_render(bodylocation, body3Dlocation_list3, Constants::minval, Constants::maxval, Constants::alpha); //gai
            body3Dlocation_mtx3.unlock();

            vr.run_only_image_content();
            // get pointer which point to the CT volume render result
            unsigned char* CT = vr.volren.img_data_from_core;
            // blend the color image and the CT volume render result to get the final blend result in variant result
            color_mtx3.lock();
            getResult(CT, color_image3, vr.volren.framebuffer_size.x, vr.volren.framebuffer_size.y);
            color_mtx3.unlock();
            //StaticFunction::save_mat_image(result, Constants::save_dir + "result_" + std::to_string(save_file_id) + ".png");

            //show final blend result
            cv::Mat resized_result;
            cv::resize(result, resized_result, cv::Size(960, 540));
            cv::imshow(Constants::window_name, resized_result);
            cv::moveWindow(Constants::window_name, 1000, 100);
            // Dynamically update the window title
            std::string dynamic_title = Constants::window_name + " " + std::to_string(save_file_id);
            cv::setWindowTitle(Constants::window_name, dynamic_title);  // 更新标题
            cv::waitKey(30);
            // print time in each code segment
            timer.printStatistics();
            //stbi_flip_vertically_on_write(1);
            //std::string fn = Constants::save_dir + "CT_" + std::to_string(save_file_id) + ".png";
            //int stride = (vr.volren.framebuffer_size.x % 4) + (vr.volren.framebuffer_size.x * 3);
            //bool status = stbi_write_png(fn.c_str(), vr.volren.framebuffer_size.x, vr.volren.framebuffer_size.y, 3, CT, stride);
        }
        t1.join();
        t2.join();
    }
}
void Work::run_multi_thread2(GetSample& sample) {
    k4a::capture capture;

    std::vector<std::vector<float>> body3Dlocation_list_front;
    body3Dlocation_list_front.resize(3);
    body3Dlocation_list_front[0] = { 11.4616f, 3.58519f, 134.144f };
    body3Dlocation_list_front[1] = { -24.4033f, -35.7827f, 150.218f };
    body3Dlocation_list_front[2] = { -56.8212f, 25.2224f, 142.874f };
    int first_run = 1;
    if (state == DYNAMIC) {
        std::thread t1(&Work::loop_get_body_location, this);
        std::thread t2(&Work::loop_get_3D_body_location, this);
        // 创建窗口（在循环外创建一次）,大小为960*540
        cv::namedWindow(Constants::window_name, cv::WINDOW_AUTOSIZE);
        cv::resizeWindow(Constants::window_name, Constants::window_width, Constants::window_height);
        cv::moveWindow(Constants::window_name, Constants::window_location_x, Constants::window_location_y);
        while (1) {
            if ((*device).get_capture(&capture, std::chrono::milliseconds(5000))) {
                k4a::image color_image_tmp = capture.get_color_image();
                save_file_id++;
                std::cout << "get valid image id: " + std::to_string(save_file_id) + "   *****************************************************************************" << std::endl;

                bodylocation = Constants::body_location; 
                body3Dlocation_mtx3.lock();
                if (body3Dlocation_list3.size() != 3) {
                    body3Dlocation_list3 = body3Dlocation_list_front;
                }
                else {
                    body3Dlocation_list_front = body3Dlocation_list3;
                }
                // start volume rendering
                vr.run_only_render(bodylocation, body3Dlocation_list3, Constants::minval, Constants::maxval, Constants::alpha); //gai
                body3Dlocation_mtx3.unlock();

                vr.run_only_image_content();
                // get pointer which point to the CT volume render result
                unsigned char* CT = vr.volren.img_data_from_core;
                // blend the color image and the CT volume render result to get the final blend result in variant result

                //getResult(CT, color_image_tmp, vr.volren.framebuffer_size.x, vr.volren.framebuffer_size.y);
                //
                //result_with_cone = StaticFunction::addCone(color_image_tmp);
                //
                //getResult2(result_with_cone, result);
                  // 创建异步任务来执行getResult和addCone
                auto addConeFuture = std::async(std::launch::async, StaticFunction::addCone, color_image_tmp);
                auto getResultFuture = std::async(std::launch::async, &Work::getResult, this, CT, std::ref(color_image_tmp), vr.volren.framebuffer_size.x, vr.volren.framebuffer_size.y);

                // 获取上一帧的结果并执行getResult2
                if (previous_result_ready) {
                    getResult2(previous_result_with_cone, previous_result);
                }

                // 等待当前帧的getResult和addCone完成
                getResultFuture.wait();
                result_with_cone = addConeFuture.get();

                // 保存当前帧的结果供下一帧使用
                previous_result = result;
                previous_result_with_cone = result_with_cone;
                previous_result_ready = true;

                //show final blend result
                cv::Mat resized_result;
                cv::resize(result2, resized_result, cv::Size(Constants::window_width, Constants::window_height));
                cv::imshow(Constants::window_name, resized_result);
                std::string dynamic_title = Constants::window_name + " " + std::to_string(save_file_id);
                cv::setWindowTitle(Constants::window_name, dynamic_title);  // 更新标题
                cv::waitKey(30);
                // print time in each code segment
                timer.printStatistics();
            }
        }
        t1.join();
        t2.join();
    }
    
}

void Work::getResult2(const cv::Mat& result_with_cone, const cv::Mat& result) {
    timer.start("Code Segment getResult2");
    // Get the width and height from the result image
    int width = result.cols;
    int height = result.rows;

    // Check if the sizes of the images match
    if (result_with_cone.cols != width || result_with_cone.rows != height) {
        std::cerr << "result_with_cone size does not match result size" << std::endl;
        // Handle this case as needed
        return;
    }
    else {
        std::cout << "result_with_cone size matches result size" << std::endl;
    }

    // Prepare the alpha channel based on the luminance and gamma correction of result_with_cone
    float gamma = 0.4f;
    cv::Mat alpha_channel(height, width, CV_8UC1);

    // Compute the alpha channel from result_with_cone
    for (int y = 0; y < height; y++) {
        const unsigned char* cone_row = result_with_cone.ptr<unsigned char>(y);
        unsigned char* alpha_row = alpha_channel.ptr<unsigned char>(y);
        for (int x = 0; x < width; x++) {
            int idx_cone = x * 3;
            unsigned char b = cone_row[idx_cone + 0];
            unsigned char g = cone_row[idx_cone + 1];
            unsigned char r = cone_row[idx_cone + 2];

            // Compute grayscale value using the luminance formula
            unsigned char gray = static_cast<unsigned char>(0.299f * r + 0.587f * g + 0.114f * b);

            // Normalize to [0,1] and apply gamma correction
            float alpha = powf(gray / 255.0f, gamma);

            // Convert back to [0,255] and store in the alpha channel
            alpha_row[x] = static_cast<unsigned char>(alpha * 255.0f);
        }
    }

    // Perform alpha blending
    for (int y = 0; y < height; y++) {
        // Pointers to the current row in result_with_cone, result, result2, and alpha_channel
        const unsigned char* cone_row = result_with_cone.ptr<unsigned char>(y);
        const unsigned char* result_row = result.ptr<unsigned char>(y);
        unsigned char* result2_row = result2.ptr<unsigned char>(y);
        const unsigned char* alpha_row = alpha_channel.ptr<unsigned char>(y);

        for (int x = 0; x < width; x++) {
            int idx_cone = x * 3;
            int idx_result = x * 3;
            int idx_result2 = x * 3;

            // Foreground color (result_with_cone)
            float f_b = cone_row[idx_cone + 0] / 255.0f;
            float f_g = cone_row[idx_cone + 1] / 255.0f;
            float f_r = cone_row[idx_cone + 2] / 255.0f;
            float f_a = alpha_row[x] / 255.0f;

            // Background color (result)
            float b_b = result_row[idx_result + 0] / 255.0f;
            float b_g = result_row[idx_result + 1] / 255.0f;
            float b_r = result_row[idx_result + 2] / 255.0f;
            // Note: We can ignore the background alpha channel if present

            // Perform alpha blending
            float out_b = f_b * f_a + b_b * (1.0f - f_a);
            float out_g = f_g * f_a + b_g * (1.0f - f_a);
            float out_r = f_r * f_a + b_r * (1.0f - f_a);

            // Store the result in result2
            result2_row[idx_result2 + 0] = static_cast<unsigned char>(out_b * 255.0f);
            result2_row[idx_result2 + 1] = static_cast<unsigned char>(out_g * 255.0f);
            result2_row[idx_result2 + 2] = static_cast<unsigned char>(out_r * 255.0f);
        }
    }
	timer.stop("Code Segment getResult2");
}





int Work::resort_3D_bodylocation_list(std::vector<std::vector<float>>& body3Dlocation_list) {
	/*
		判断3D_bodylocation_list的合法性。返回0代表不合法，返回1代表合法。
	*/
    using namespace std;
	timer.start("Code Segment resort");
	// Remove invalid locations
	StaticFunction::removeInvalidLocations(body3Dlocation_list, { -10001.0f, -10001.0f, -10001.0f });
	if (body3Dlocation_list.size() < 3) {
		body3Dlocation_list.clear();
		return 0;
	}
    

    vector<float> chest1 = Constants::volume_point_one;
    vector<float> chest2 = Constants::volume_point_two;
    vector<float> chest3 = Constants::volume_point_three;

    // Compute the sides of the chest triangle
    float chest_side1 = StaticFunction::distance(chest1, chest2); // between chest1 and chest2
    float chest_side2 = StaticFunction::distance(chest2, chest3); // between chest2 and chest3
    float chest_side3 = StaticFunction::distance(chest3, chest1); // between chest3 and chest1

    // Store the chest sides in a vector for easier comparison
    vector<float> chest_sides = { chest_side1, chest_side2, chest_side3 };
    vector<vector<float>> body3Dlocation_list_tmp;
    float tolerance = Constants::tolerance; // Allowable relative deviation (10%)
	float min_tolerance = Constants::tolerance; // Minimum deviation
    size_t n = body3Dlocation_list.size();
    bool match_found = false;
    // Iterate over all combinations of three distinct points
    for (size_t i = 0; i < n - 2; ++i) {
        for (size_t j = i + 1; j < n - 1; ++j) {
            for (size_t k = j + 1; k < n; ++k) {
                vector<vector<float>> points = {
                    body3Dlocation_list[i],
                    body3Dlocation_list[j],
                    body3Dlocation_list[k]
                };

                vector<int> perm = { 0, 1, 2 };

                // Consider all permutations of the three points
                do {
                    vector<float> a = points[perm[0]];
                    vector<float> b = points[perm[1]];
                    vector<float> c = points[perm[2]];
					// Check if the points are in the same order as the chest points
					bool isSameOrder = StaticFunction::isSameOrder(a, b, c, chest1, chest2, chest3);
                    if (isSameOrder == false) continue;
                    // Compute sides of triangle abc in the same order as chest_sides
                    float side1 = StaticFunction::distance(a, b); // corresponds to chest_side1
                    float side2 = StaticFunction::distance(b, c); // corresponds to chest_side2
                    float side3 = StaticFunction::distance(c, a); // corresponds to chest_side3

                    // Store the sides in order
                    vector<float> body_sides = { side1, side2, side3 };

                    // Compute scaling factors for corresponding sides
                    float s1 = body_sides[0] / chest_sides[0];
                    float s2 = body_sides[1] / chest_sides[1];
                    float s3 = body_sides[2] / chest_sides[2];
					float max_s = max({ s1, s2, s3 });
                    // Compute mean scaling factor
                    float s_mean = (s1 + s2 + s3) / 3.0f;

                    // Compute relative deviations
                    float dev1 = fabs(s1 - s_mean) / s_mean;
                    float dev2 = fabs(s2 - s_mean) / s_mean;
                    float dev3 = fabs(s3 - s_mean) / s_mean;

                    float max_dev = max({ dev1, dev2, dev3 });

					std::cout << "min_tolerance: " << min_tolerance << std::endl;
					std::cout << "s1: " << s1 << " s2: " << s2 << " s3: " << s3 << " s_mean: " << s_mean << " dev1: " << dev1 << " dev2: " << dev2 << " dev3: " << dev3 << " max_dev: " << max_dev << std::endl;
					// Check if the triangle is similar to the chest triangle
                    if (max_dev < tolerance && max_dev < min_tolerance && max_s <= 1000) {
						min_tolerance = max_dev;
                        body3Dlocation_list_tmp = { a, b, c };
                        //body3Dlocation_list = { a, b, c };
                        match_found = true;
                    }
                } while (next_permutation(perm.begin(), perm.end()));
            }
        }
    }
    if (match_found) {
        body3Dlocation_list = body3Dlocation_list_tmp;
        std::cout << "the best tolerance is " << min_tolerance << std::endl;
        // sort the body3Dlocation_list in descending order of x
        //std::sort(body3Dlocation_list.begin(), body3Dlocation_list.end(), [](const vector<float>& a, const vector<float>& b) {
        //    return a[0] > b[0];
        //    });
        timer.stop("Code Segment resort");
        return 1;
    }
    // No similar triangle found
    body3Dlocation_list.clear();
    timer.stop("Code Segment resort");
    return 0;

}

void Work::getResult(unsigned char* CT, k4a::image& color_image, int CT_width, int CT_height) {
	/*
		we implement it without opencv, because using opencv is too slow to realize real-time display.
		CT length :width * height * 3
		we ensure that color_image's width / height is equal to CT's width / height
		first I should scale CT to color_image.
		Then I should overlay CT on color_image and get the unsigned char* result.
		Pay atteention that result is a member of Work class.we don't need to return it.
		we don't need to new the result every time.Only need to new it once.
		and than display it using opencv.
	*/
    // 获取color_image的宽高
    timer.start("Code Segment get result");
    int width = color_image.get_width_pixels();
    int height = color_image.get_height_pixels();

    // 获取color_image的数据
    uint8_t* color_data = color_image.get_buffer();



    // 分配大小为width * height * 3的CT_image缓冲区，用于存储调整尺寸后的CT图像
    unsigned char* CT_resized = CT;

	if (CT_width != width || CT_height != height) {
		std::cerr << "CT image size does not match color image size" << std::endl;
    }
    else {
        std::cout << "CT image size match color image size" << std::endl;
    }


    // 计算亮度（灰度值），并进行gamma校正，作为alpha通道
    float gamma = 0.4f;
    unsigned char* alpha_channel = new unsigned char[width * height];

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = (y * width + x);
            unsigned char r = CT_resized[idx * 3 + 2];
            unsigned char g = CT_resized[idx * 3 + 1];
            unsigned char b = CT_resized[idx * 3 + 0];
            // 计算灰度值
            unsigned char gray = (unsigned char)(0.299f * r + 0.587f * g + 0.114f * b);
            // 归一化到[0,1]并进行gamma校正
            float alpha = powf(gray / 255.0f, gamma);
            // 转换回[0,255]
            alpha_channel[idx] = (unsigned char)(alpha * 255.0f);
        }
    }

    // 开始进行alpha混合
    uint8_t* result_data = result.data;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = (y * width + x);

            // 前景色（CT图像）
            float f_b = CT_resized[idx * 3 + 0] / 255.0f;
            float f_g = CT_resized[idx * 3 + 1] / 255.0f;
            float f_r = CT_resized[idx * 3 + 2] / 255.0f;
            float f_a = alpha_channel[idx] / 255.0f;

            // 背景色（color_image）
            float b_b = color_data[idx * 4 + 0] / 255.0f;
            float b_g = color_data[idx * 4 + 1] / 255.0f;
            float b_r = color_data[idx * 4 + 2] / 255.0f;
            // 注意：color_image可能有alpha通道，但这里我们不需要

            // 进行alpha混合
            float out_b = f_b * f_a + b_b * (1.0f - f_a);
            float out_g = f_g * f_a + b_g * (1.0f - f_a);
            float out_r = f_r * f_a + b_r * (1.0f - f_a);

            // 结果存入result
            result_data[idx * 3 + 0] = (unsigned char)(out_b * 255.0f);
            result_data[idx * 3 + 1] = (unsigned char)(out_g * 255.0f);
            result_data[idx * 3 + 2] = (unsigned char)(out_r * 255.0f);
        }
    }

    // 释放临时内存
    delete[] alpha_channel;
    timer.stop("Code Segment get result");

	return;
}

int Work::get_body_location(k4a::image& color_image)
{
    /*
     by hy
    */
	timer.start("Code Segment get_body_location");
    using namespace std;
    using namespace cv;
    // 创建一个向量来存储椭圆的中心点
    vector<pair<int, int>> centers;
    int width = color_image.get_width_pixels();
    int height = color_image.get_height_pixels();
    uint8_t* buffer = color_image.get_buffer();
    // 读取图像
    // 创建一个包含图像数据的 Mat（假设图像格式为 BGRA）
    cv::Mat img_bgra(height, width, CV_8UC4, buffer);

    // 如果需要将图像从 BGRA 转换为 BGR
    cv::Mat img;
    cv::cvtColor(img_bgra, img, cv::COLOR_BGRA2BGR);
    if (img.empty()) {
        //cout << "无法打开或找到图像" << endl;
        return 0;
    }

    // 转换为灰度图像
    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);

    // 高斯模糊（减少噪声）
    Mat blurred;
    GaussianBlur(gray, blurred, Size(5, 5), 0);

    // 自适应阈值分割
    Mat adaptive_thresh;
    adaptiveThreshold(blurred, adaptive_thresh, 255,
        ADAPTIVE_THRESH_GAUSSIAN_C,
        THRESH_BINARY_INV, 11, 2);

    // 形态学操作
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    Mat morph = adaptive_thresh.clone();
    // 形态学开运算（消除小噪声）
    // morphologyEx(adaptive_thresh, morph, MORPH_OPEN, kernel);

    // 形态学闭运算（填充内部空洞）
    morphologyEx(morph, morph, MORPH_CLOSE, kernel);

    // 寻找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(morph, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 绘制轮廓
    Mat contour_img = img.clone();
    drawContours(contour_img, contours, -1, Scalar(255, 0, 0), 2);

    // 设置长轴长度的最大值
    double max_axis = 120.0;  // 根据您的需求设置
    double max_e = 0.85;
    double fit_quality_threshold = 0.002;

    // 拟合椭圆并绘制
    Mat ellipse_img = img.clone();
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = contourArea(contours[i]);
        if (area > 500 && contours[i].size() >= 5) {
            try {
                RotatedRect ellipseRect = fitEllipse(contours[i]);
                Point2f center = ellipseRect.center;
                Size2f axes = ellipseRect.size;
                float angle = ellipseRect.angle;

                double axis1 = axes.width;
                double axis2 = axes.height;
                if (axis1 < axis2) {
                    swap(axis1, axis2);
                }
                double e = sqrt(1.0 - pow(axis2 / axis1, 2));
                //cout << "离心率: " << e << "  长轴长度: " << axis1 << endl;

                // 检查轴长度是否为正
                if (axes.width > 0 && axes.height > 0) {
                    // 应用长轴长度的最大值限制
                    double mean_distance = StaticFunction::evaluate_fit_quality(contours[i], ellipseRect);
                    //cout << "拟合质量: " << mean_distance << endl;
                    if (axis1 <= max_axis && e < max_e && mean_distance <= fit_quality_threshold) {
                        //cout << "检测到有效椭圆，中心点: (" << center.x << ", " << center.y << ")" << endl;
                        ellipse(ellipse_img, ellipseRect, Scalar(0, 255, 0), 2);

                        // 将中心点存储到向量中
                        centers.push_back(make_pair(static_cast<int>(center.x), static_cast<int>(center.y)));
                    }
                }
            }
            catch (cv::Exception& e) {
                //cout << "拟合椭圆时出错: " << e.what() << endl;
            }
        }
    }

    // 对中心点进行排序（按照x坐标升序排序，如果需要按照其他方式排序，可以修改排序条件）
    sort(centers.begin(), centers.end(), [](const pair<int, int>& a, const pair<int, int>& b) {
        return a.first < b.first;
        });


    // 输出排序后的中心点
    cout << "排序后的椭圆中心点：" << endl;
    for (const auto& center : centers) {
        cout << "(" << center.first << ", " << center.second << ")" << endl;
    }

	bodylocation_list = centers;
	timer.stop("Code Segment get_body_location");
    if (centers.size() != 3) return 0;

    return 1;
}

void Work::loop_get_body_location() {
	k4a::capture capture;
    while (1) {
        ready2 = false;
        std::unique_lock<std::mutex> lock(mtx2);
        while(!ready2)
			cv2.wait(lock);
        if ((*device).get_capture(&capture, std::chrono::milliseconds(5000))) {
            timer.start("Code Segment capture");
            color_mtx2.lock();
            color_image = capture.get_color_image();
            color_mtx2.unlock();
            depth_mtx2.lock();
            depth_image = capture.get_depth_image();
            depth_mtx2.unlock();
        }
        frame_id++;
        std::cout << "start frame: " << frame_id << " ------------------------------------------------------------------------------------------" << std::endl;
		bodylocation_mtx2.lock();
        get_body_location(color_image);
		bodylocation_mtx2.unlock();
        std::cout << "loop end" << std::endl;
    }
}

void Work::loop_get_3D_body_location() {
    bool flag = false;
    while (1) {
        bodylocation_mtx2.lock();
        if (bodylocation_list.size() >= 3) {
            flag = true;
            // 将color_image的数据传递给color_image2
            color_mtx2.lock();
            color_image2 = color_image;
            color_mtx2.unlock();
            // 将depth_image的数据传递给depth_image2
            depth_mtx2.lock();
            depth_image2 = depth_image;
            depth_mtx2.unlock();
			// 将bodylocation_list的数据传递给bodylocation_list2
			bodylocation_list2 = bodylocation_list;
        }
        else {
			flag = false;
        }
		bodylocation_mtx2.unlock();

        ready2 = true;
		cv2.notify_one();
        if (flag == false) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
		timer.start("Code Segment parall");
        body3Dlocation_list2.clear();
        /*
        //method1
		for (int i = 0; i < bodylocation_list2.size(); i++) {
			get3dcords.get_cords_with_depth_image(bodylocation_list2[i].first, bodylocation_list2[i].second, depth_image2, body3Dlocation_list2);
		}
        //method2
        body3Dlocation_list2.resize(bodylocation_list2.size());
        for (int i = 0; i < bodylocation_list2.size(); i++) {
			body3Dlocation_list2[i] = get3dcords.get_cords_with_depth_image2(bodylocation_list2[i].first, bodylocation_list2[i].second, depth_image2);
        }
    
        //method3
        body3Dlocation_list2.resize(bodylocation_list2.size());

#pragma omp parallel for
        for (int i = 0; i < bodylocation_list2.size(); i++) {
            // 获取结果
            auto result = get3dcords.get_cords_with_depth_image2(
                bodylocation_list2[i].first,
                bodylocation_list2[i].second,
                depth_image2
            );
            // 将结果存储在预先分配的向量中
            body3Dlocation_list2[i] = result;
        }
        */
        //method4
        // 假设 bodylocation_list2 和 body3Dlocation_list2 已经定义
        // 预先分配结果向量的大小
        body3Dlocation_list2.resize(bodylocation_list2.size());

        // 获取硬件支持的并发线程数
        unsigned int num_threads = std::thread::hardware_concurrency();

        // 定义每个线程需要处理的任务量
        int total_tasks = bodylocation_list2.size();
        int chunk_size = (total_tasks + num_threads - 1) / num_threads; // 向上取整

        // 定义线程函数
        auto thread_func = [&](int start_index, int end_index) {
            for (int i = start_index; i < end_index; ++i) {
                auto result = get3dcords.get_cords_with_depth_image2(
                    bodylocation_list2[i].first,
                    bodylocation_list2[i].second,
                    depth_image2
                );
                body3Dlocation_list2[i] = result;
            }
            };

        // 创建并启动线程
        std::vector<std::thread> threads;
        for (unsigned int t = 0; t < num_threads; ++t) {
            int start_index = t * chunk_size;
            int end_index = std::min(start_index + chunk_size, total_tasks);
            if (start_index < end_index) {
                threads.emplace_back(thread_func, start_index, end_index);
            }
        }

        // 等待所有线程完成
        for (auto& th : threads) {
            th.join();
        }
        timer.stop("Code Segment parall");
        timer.start("Code Segment resort in parell");
		int detect_state = resort_3D_bodylocation_list(body3Dlocation_list2);
        timer.stop("Code Segment resort in parell");
        if (detect_state == 1) {

			color_mtx3.lock();
            color_image3 = color_image2;
			color_mtx3.unlock();

			body3Dlocation_mtx3.lock();
			body3Dlocation_list3 = body3Dlocation_list2;
			body3Dlocation_mtx3.unlock();

            ready3 = true;
            cv3.notify_one();
        }


    }
}

void Work::getResultWithMat(unsigned char* CT, cv::Mat& color_image, int CT_width, int CT_height) {
    // Start the timer
    timer.start("Code Segment get result");

    // Get the width and height from the color_image
    int width = color_image.cols;
    int height = color_image.rows;

    // Check if the CT image size matches the color image size
    if (CT_width != width || CT_height != height) {
        std::cerr << "CT image size does not match color image size" << std::endl;
        // Handle this case as needed, e.g., resize CT image to match color_image
        return;
    }
    else {
        std::cout << "CT image size matches color image size" << std::endl;
    }

    // Prepare the alpha channel based on the CT image's luminance and gamma correction
    float gamma = 0.4f;
    unsigned char* alpha_channel = new unsigned char[width * height];

    // Compute the alpha channel from the CT image
    for (int y = 0; y < height; y++) {
        unsigned char* alpha_row = alpha_channel + y * width;
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            unsigned char b = CT[idx * 3 + 0];
            unsigned char g = CT[idx * 3 + 1];
            unsigned char r = CT[idx * 3 + 2];
            // Compute the grayscale value
            unsigned char gray = static_cast<unsigned char>(0.299f * r + 0.587f * g + 0.114f * b);
            // Normalize to [0,1] and apply gamma correction
            float alpha = powf(gray / 255.0f, gamma);
            // Convert back to [0,255]
            alpha_row[x] = static_cast<unsigned char>(alpha * 255.0f);
        }
    }

    // Perform alpha blending
    for (int y = 0; y < height; y++) {
        // Pointers to the current row in CT, color_image, result, and alpha_channel
        unsigned char* ct_row = CT + y * width * 3;
        unsigned char* color_row = color_image.ptr<unsigned char>(y);
        unsigned char* result_row = result.ptr<unsigned char>(y);
        unsigned char* alpha_row = alpha_channel + y * width;
        for (int x = 0; x < width; x++) {
            int idx_ct = x * 3;
            int idx_color = x * 3;
            int idx_result = x * 4;

            // Foreground color (CT image)
            float f_b = ct_row[idx_ct + 0] / 255.0f;
            float f_g = ct_row[idx_ct + 1] / 255.0f;
            float f_r = ct_row[idx_ct + 2] / 255.0f;
            float f_a = alpha_row[x] / 255.0f;

            // Background color (color_image)
            float b_b = color_row[idx_color + 0] / 255.0f;
            float b_g = color_row[idx_color + 1] / 255.0f;
            float b_r = color_row[idx_color + 2] / 255.0f;
            // Note: We can ignore the background alpha channel if present

            // Perform alpha blending
            float out_b = f_b * f_a + b_b * (1.0f - f_a);
            float out_g = f_g * f_a + b_g * (1.0f - f_a);
            float out_r = f_r * f_a + b_r * (1.0f - f_a);

            // Store the result
            result_row[idx_result + 0] = static_cast<unsigned char>(out_b * 255.0f);
            result_row[idx_result + 1] = static_cast<unsigned char>(out_g * 255.0f);
            result_row[idx_result + 2] = static_cast<unsigned char>(out_r * 255.0f);
            result_row[idx_result + 3] = 255; // Set alpha to opaque
        }
    }

    // Release the temporary memory
    delete[] alpha_channel;
    timer.stop("Code Segment get result");

    return;
}