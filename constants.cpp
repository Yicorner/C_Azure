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
    std::string Shaderfile = "D:\\data\\project\\VisualStudio\\C_Azure\\shader\\VolumeRenderer4_ignore_small_pixel.cs";
    std::string Datafile = "D:\\data\\project\\VisualStudio\\Volume-Renderer-1.0\\Volume-Renderer-1.0\\fakeman.raw";

    std::string from_file_color_filename = "D:\\data\\project\\VisualStudio\\C_Azure\\FROM_FILE\\color_5.png";
    std::string from_file_depth_filename = "D:\\data\\project\\VisualStudio\\C_Azure\\FROM_FILE\\depth_5.raw";

    std::string save_dir = "D:/data/project/VisualStudio/C_Azure/verify_color_image2/";
    std::string window_name = "Blended Image";
    int if_multi_thread = 3;
    int if_log_file = 1;
    int if_real_time_display = 1;
    int if_origin_glfw_manager = 1;

	int width = 1920;
	int height = 1080;

    int window_width = 1920;
	int window_height = 1080;

	int window_location_x = 0;
	int window_location_y = 0;

    int if_teapot = 1;
    float cube_vertices[] = {
      -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,
       0.5f, -0.5f, -0.5f,  1.0f, 0.0f,
       0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
       0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
      -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
      -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,

      -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
       0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
       0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
       0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
      -0.5f,  0.5f,  0.5f,  0.0f, 1.0f,
      -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,

      -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
      -0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
      -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
      -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
      -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
      -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

       0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
       0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
       0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
       0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
       0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
       0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

      -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
       0.5f, -0.5f, -0.5f,  1.0f, 1.0f,
       0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
       0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
      -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
      -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,

      -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
       0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
       0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
       0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
      -0.5f,  0.5f,  0.5f,  0.0f, 0.0f,
      -0.5f,  0.5f, -0.5f,  0.0f, 1.0f
    };
    int cube_vertices_size = 180;

    std::string graphic_fragement_shader = "D:\\data\\project\\VisualStudio\\C_Azure\\shader\\6.2.coordinate_systems.fs";
    std::string graphic_vertex_shader = "D:\\data\\project\\VisualStudio\\C_Azure\\shader\\6.2.coordinate_systems.vs";

    std::string texture_fragement_shader1 = "D:\\data\\project\\VisualStudio\\MarkerDetector\\teapot.png";
    std::string texture_fragement_shader2 = "D:\\data\\project\\VisualStudio\\MarkerDetector\\image.jpg";

    glm::vec3 voxel_size = { 0, 0, 0 };
    glm::ivec3 tex3D_dim = { 0, 0 ,0 };
}
