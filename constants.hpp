// Constants.hpp
#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <string>
#include <vector>
#include "AllEnum.hpp"
namespace Constants {
	BodyLocation body_location = BodyLocation::TRUEMANLEG;
	State state = State::DYNAMIC;
	RealTimeDisplayState real_time_display_state = RealTimeDisplayState::ONLY_COLOR;

	float tolerance = 0.2f;
	constexpr int minval = 150;
	constexpr int maxval = 170;
	constexpr float alpha = 0.181f;
	std::vector<float> volume_point_one = { -0.288889f, 0.197531f, 0.330864f };
	std::vector<float> volume_point_two = { -0.264198f, 0.022222f, 0.271605f };
	std::vector<float> volume_point_three = { -0.320988f, 0.172840f, 0.051852f };
	std::string Shaderfile = "D:\\data\\project\\VisualStudio\\Volume-Renderer-1.0\\Volume-Renderer-1.0\\VolumeRenderer.cs";
	std::string Datafile = "D:\\data\\project\\VisualStudio\\Volume-Renderer-1.0\\Volume-Renderer-1.0\\trueman2.raw";

	std::string from_file_color_filename = "D:\\data\\project\\VisualStudio\\C_Azure\\FROM_FILE\\color_5.png";
	std::string from_file_depth_filename = "D:\\data\\project\\VisualStudio\\C_Azure\\FROM_FILE\\depth_5.raw";

	std::string save_dir = "D:/data/project/VisualStudio/C_Azure/verify_color_image2/";
	std::string window_name = "Blended Image";
	int if_multi_thread = 1;
	int if_log_file = 1;
	int if_real_time_display = 1;
	int if_origin_glfw_manager = 0;
}

#endif // CONSTANTS_HPP
