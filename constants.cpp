// Constants.cpp
#include "Constants.hpp"

namespace Constants {
    BodyLocation body_location = BodyLocation::ABDOMEN;
    State state = State::DYNAMIC;
    RealTimeDisplayState real_time_display_state = RealTimeDisplayState::ONLY_COLOR;

    float tolerance = 0.2f;
    const int minval = 80;
    const int maxval = 255;
    const float alpha = 0.5f;
    std::vector<float> volume_point_one = { -0.101235f, -0.180247f, -0.269136f };
    std::vector<float> volume_point_two = { -0.195062f, -0.150617f, -0.276543f };
    std::vector<float> volume_point_three = { -0.103704f, -0.177778f, -0.375309f };
    std::string Shaderfile = "D:\\data\\project\\VisualStudio\\Volume-Renderer-1.0\\Volume-Renderer-1.0\\VolumeRenderer.cs";
    std::string Datafile = "D:\\data\\project\\VisualStudio\\Volume-Renderer-1.0\\Volume-Renderer-1.0\\fakeman.raw";

    std::string from_file_color_filename = "D:\\data\\project\\VisualStudio\\C_Azure\\FROM_FILE\\color_5.png";
    std::string from_file_depth_filename = "D:\\data\\project\\VisualStudio\\C_Azure\\FROM_FILE\\depth_5.raw";

    std::string save_dir = "D:/data/project/VisualStudio/C_Azure/verify_color_image2/";
    std::string window_name = "Blended Image";
    int if_multi_thread = 2;
    int if_log_file = 1;
    int if_real_time_display = 1;
    int if_origin_glfw_manager = 0;

	int width = 1920;
	int height = 1080;
}
