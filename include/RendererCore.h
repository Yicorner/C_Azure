#ifndef RENDERERCORE_H
#define RENDERERCORE_H

#include "glad/glad.h"
#include <vector>
#include <string>
#include "glm/vec3.hpp"
#include "glm/vec2.hpp"
#include "Camera.h"
class RendererCore
{
    public:
        RendererCore();
        ~RendererCore();
        void setup();
        void render();
        Camera main_cam; // qbh
        unsigned char* img_data_from_core;
        glm::ivec2 window_size, framebuffer_size;

    private:
        friend class RendererGUI;
        void setAlpha();
        void setMinVal();
        void setMaxVal();
        void setMIP();
        void setUniforms();
        void setInitialCameraRotation();
        void setupFBO();
        void setupUBO(bool is_update = false);
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
        GLuint vol_tex3D, camera_ubo_ID, fbo_ID, fbo_texID, cs_ID, cs_programID;


};

#endif // RENDERERCORE_H
