//// Constants.hpp
//#ifndef CONSTANTS_HPP
//#define CONSTANTS_HPP
//
//#include <string>
//#include <vector>
//#include "AllEnum.hpp"
//namespace Constants {
//	//BodyLocation body_location = BodyLocation::TRUEMANLEG;
//	//State state = State::DYNAMIC;
//	//RealTimeDisplayState real_time_display_state = RealTimeDisplayState::ONLY_COLOR;
//
//	//float tolerance = 0.2f;
//	//constexpr int minval = 150;
//	//constexpr int maxval = 170;
//	//constexpr float alpha = 0.181f;
//	//std::vector<float> volume_point_one = { -0.288889f, 0.197531f, 0.330864f };
//	//std::vector<float> volume_point_two = { -0.264198f, 0.022222f, 0.271605f };
//	//std::vector<float> volume_point_three = { -0.320988f, 0.172840f, 0.051852f };
//	//std::string Shaderfile = "D:\\data\\project\\VisualStudio\\Volume-Renderer-1.0\\Volume-Renderer-1.0\\VolumeRenderer.cs";
//	//std::string Datafile = "D:\\data\\project\\VisualStudio\\Volume-Renderer-1.0\\Volume-Renderer-1.0\\trueman2.raw";
//
//	//std::string from_file_color_filename = "D:\\data\\project\\VisualStudio\\C_Azure\\FROM_FILE\\color_5.png";
//	//std::string from_file_depth_filename = "D:\\data\\project\\VisualStudio\\C_Azure\\FROM_FILE\\depth_5.raw";
//
//	//std::string save_dir = "D:/data/project/VisualStudio/C_Azure/verify_color_image2/";
//	//std::string window_name = "Blended Image";
//	//int if_multi_thread = 1;
//	//int if_log_file = 1;
//	//int if_real_time_display = 1;
//	//int if_origin_glfw_manager = 0;
//	BodyLocation body_location = BodyLocation::ABDOMEN;
//	State state = State::DYNAMIC;
//	RealTimeDisplayState real_time_display_state = RealTimeDisplayState::ONLY_COLOR;
//
//	float tolerance = 0.2f;
//	constexpr int minval = 80;
//	constexpr int maxval = 155;
//	constexpr float alpha = 0.4f;
//	std::vector<float> volume_point_one = { -0.211112f, -0.25f, 0.161112f };
//	std::vector<float> volume_point_two = { -0.281456f, -0.227778f, 0.11111f };
//	std::vector<float> volume_point_three = { -0.177778f, -0.244444f, 0.047222f };
//	std::string Shaderfile = "D:\\data\\project\\VisualStudio\\Volume-Renderer-1.0\\Volume-Renderer-1.0\\VolumeRenderer.cs";
//	std::string Datafile = "D:\\data\\project\\VisualStudio\\Volume-Renderer-1.0\\Volume-Renderer-1.0\\fakeman.raw";
//
//	std::string from_file_color_filename = "D:\\data\\project\\VisualStudio\\C_Azure\\FROM_FILE\\color_5.png";
//	std::string from_file_depth_filename = "D:\\data\\project\\VisualStudio\\C_Azure\\FROM_FILE\\depth_5.raw";
//
//	std::string save_dir = "D:/data/project/VisualStudio/C_Azure/verify_color_image2/";
//	std::string window_name = "Blended Image";
//	int if_multi_thread = 0;
//	int if_log_file = 1;
//	int if_real_time_display = 1;
//	int if_origin_glfw_manager = 0;
//}
//
//#endif // CONSTANTS_HPP


// Constants.hpp
#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <string>
#include <vector>
#include "AllEnum.hpp"
#include <glm/glm.hpp>

namespace Constants {
    extern BodyLocation body_location;
    extern State state;
    extern RealTimeDisplayState real_time_display_state;

    extern float tolerance;
    extern const int minval;
    extern const int maxval;
    extern const float alpha;
    extern std::vector<float> volume_point_one;
    extern std::vector<float> volume_point_two;
    extern std::vector<float> volume_point_three;
    extern std::string Shaderfile;
    extern std::string Datafile;

    extern std::string from_file_color_filename;
    extern std::string from_file_depth_filename;

    extern std::string save_dir;
    extern std::string window_name;
    extern int if_multi_thread;
    extern int if_log_file;
    extern int if_real_time_display;
    extern int if_origin_glfw_manager;

    extern int width;
    extern int height;

    extern int window_width;
    extern int window_height;

    extern int window_location_x;
    extern int window_location_y;

    extern int if_teapot;
    extern float cube_vertices[];
    extern int cube_vertices_size;

    extern std::string graphic_fragement_shader;
    extern std::string graphic_vertex_shader;

    extern std::string texture_fragement_shader1;
    extern std::string texture_fragement_shader2;

    extern glm::vec3 voxel_size;
    extern glm::ivec3 tex3D_dim;
};

#endif // CONSTANTS_HPP
