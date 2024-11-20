#ifndef RENDERERCORE_H
#define RENDERERCORE_H

#include "glad/glad.h"
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <memory> 
#include <iostream>

#include "Camera.h"
#include "k4a/k4a.hpp"
#include "MultiTimer.hpp"
#include "shader_m.h"
#include "constants.hpp"
#include "stb_image.h"
class RendererCore
{
    public:
        RendererCore(k4a::device& device, k4a_device_configuration_t& config);
        ~RendererCore();
        void setup();
        void MySetup();
        void MySetupCube();
        void render();
        void render2();
        void render_cube();
        void setThreShold();
        void setSegmentation();
		void setPlane();
        Camera main_cam;
        unsigned char* img_data_from_core;
        glm::ivec2 window_size, framebuffer_size;
        GLuint vol_tex3D, camera_ubo_ID, fbo_ID, fbo_texID, cs_ID, cs_programID;
        void setupUBO(bool is_update = false);
        k4a::device* device; 
        k4a_device_configuration_t* config; 
		k4a::image color_image; 
        GLuint textureID;
        std::vector<float> vertices;
        GLuint VAO, VBO, shaderProgram;
        GLuint ssbo;
		glm::mat4 model, projection, view;
        int if_detected_marker;
		GLuint texture1, texture2;
        Shader  ourShader;
        float threshold;
        float plane_a, plane_b, plane_c;
        bool segmentation;

    private:
        MultiTimer& timer = MultiTimer::getInstance();
        friend class RendererGUI;
        void setAlpha();
        void setMinVal();
        void setMaxVal();
        void setMIP();
        void setUniforms();
        void setInitialCameraRotation();
        void setupFBO();
        void readVolumeData(std::string fn);
        void getImageContent();
        bool checkRawInfFile(std::string fn);
        bool saveImage(std::string fn, std::string ext);
        bool loadShader(std::string fn, bool reload);
        bool createShader(std::string fn, bool reload);
        bool createShaderProgram();
        
        std::vector<float> histogram;
        std::string loaded_dataset, loaded_shader, msg, title;
        float alpha_scale, kerneltime_sum;
        int workgroups_x, workgroups_y, datasize_bytes, min_val, max_val, max_dataset_val, min_dataset_val;
        bool use_mip, rotate_to_bottom, rotate_to_top;
        glm::vec3 voxel_size;
        glm::ivec3 tex3D_dim;

        // 顶点着色器源代码
        const char* vertexShaderSource = R"(
    #version 430 core
    layout (location = 0) in vec2 aPos;
    void main()
    {
        gl_Position = vec4(aPos, 0.0, 1.0);
    }
)";

        // 片段着色器源代码
        const char* fragmentShaderSource = R"(
    #version 430 core
    out vec4 FragColor;
    uniform vec3 lineColor;
    void main()
    {
        FragColor = vec4(lineColor, 1.0);
    }
)";

};

#endif // RENDERERCORE_H
