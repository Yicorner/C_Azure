#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdint>
#include <iostream>

#include "glad/glad.h"
#include "RendererCore.h"
#include "pvm2raw.h"
#include "stb_image_write.h"

RendererCore::RendererCore(k4a::device& device, k4a_device_configuration_t& config) : main_cam(62), histogram(256,0.0f)
{
    voxel_size = glm::vec3(1.0f, 1.0f, 1.0f);
    tex3D_dim = glm::vec3(0, 0, 0);
    cs_ID = cs_programID = 0;
    alpha_scale = 1;
    min_val = 0;
    max_val = 0;
    datasize_bytes = -1;
    kerneltime_sum = 0.0;
    camera_ubo_ID = 0;
    workgroups_x = workgroups_y = 0;
    use_mip = rotate_to_bottom = rotate_to_top = false;
    img_data_from_core = nullptr;
    this->device = &device;
	this->config = &config;
    vertices = {};

    threshold = 0.0f;
	if_detected_marker = 0;
    segmentation = 0;
	plane_a = 0;
	plane_b = 0;
	plane_c = 0;

    // set model, projection, view matrix
    glm::mat4 cubePoseMatrix = glm::mat4(
        0.9607884553950903, -0.07179183067393458, -0.2678273268880558, -0.1956209865621451,
        -0.07682767432524706, -0.9970093693783699, -0.008356185105593715, -0.06246427847121779,
        -0.2664264484567742, 0.02860507682615588, -0.9634306914057075, 0.2517965325514948,
        0, 0, 0, 1
    );

    cubePoseMatrix = glm::transpose(cubePoseMatrix);
    model = cubePoseMatrix;

    float fov_x = 92.75274f;
    float fov_y = 61.11177f;
    float fov_x_rad = glm::radians(fov_x);
    float fov_y_rad = glm::radians(fov_y);
    float aspect_ratio = tan(fov_x_rad / 2.0f) / tan(fov_y_rad / 2.0f);
    projection = glm::perspective(fov_y_rad, aspect_ratio, 0.001f, 100.0f);


    glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, -0.0f);
    glm::vec3 cameraTarget = glm::vec3(0.0f, 0.0f, 1.0f); // 朝向正 z 轴
    glm::vec3 cameraUp = glm::vec3(0.0f, -1.0f, 0.0f); // y 轴向下
    view = glm::lookAt(cameraPos, cameraTarget, cameraUp);
}

RendererCore::~RendererCore()
{
    if(cs_programID)
        glDeleteProgram(cs_programID);
	delete[] img_data_from_core;
}

void RendererCore::setup()
{
    setupFBO();
	glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_ID); //Bind Framebuffer to read from
	glReadBuffer(GL_COLOR_ATTACHMENT0); //Read from Color Attachment 0
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0); //Bind Framebuffer to draw to
	glDrawBuffer(GL_BACK); //Draw to Back Buffer

    //Setup a texture and load data later..
	glGenTextures(1, &vol_tex3D); //Generate a 3D Texture
    
}

void RendererCore::MySetup()
{

    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glBindTexture(GL_TEXTURE_2D, 0);
    GLuint textureLocation = glGetUniformLocation(cs_programID, "inputTexture");
    glUniform1i(textureLocation, 2);

    glGenBuffers(1, &ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo); // Binding point 1 as per compute shader
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void RendererCore::MySetupCube()
{
    ourShader.initShader(Constants::graphic_vertex_shader.c_str(), Constants::graphic_fragement_shader.c_str());
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * Constants::cube_vertices_size, Constants::cube_vertices, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // texture coord attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glGenTextures(1, &texture1);
    glBindTexture(GL_TEXTURE_2D, texture1);
    // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // load image, create texture and generate mipmaps
    int width, height, nrChannels;
    stbi_set_flip_vertically_on_load(true); // tell stb_image.h to flip loaded texture's on the y-axis.
    unsigned char* data = stbi_load(Constants::texture_fragement_shader1.c_str(), &width, &height, &nrChannels, 0);
    if (data)
    {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    else
    {
        std::cout << "Failed to load texture" << std::endl;
    }
    stbi_image_free(data);
    // texture 2
    // ---------
    glGenTextures(1, &texture2);
    glBindTexture(GL_TEXTURE_2D, texture2);
    // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // load image, create texture and generate mipmaps
    data = stbi_load(Constants::texture_fragement_shader2.c_str(), &width, &height, &nrChannels, 0);
    if (data)
    {
        // note that the awesomeface.png has transparency and thus an alpha channel, so make sure to tell OpenGL the data type is of GL_RGBA
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    else
    {
        std::cout << "Failed to load texture" << std::endl;
    }
    stbi_image_free(data);
    ourShader.use();
    ourShader.setInt("texture1", 5);
    ourShader.setInt("texture2", 6);
    glUseProgram(cs_programID);
}

bool RendererCore::checkRawInfFile(std::string fn)
{
    std::ifstream inf_file;
    inf_file.open(fn + ".inf");
    if(inf_file)
        return true;
    else
        return false;
}

void RendererCore::setAlpha()
{
    if(cs_programID)
        glUniform1f(0, alpha_scale);
}

void RendererCore::setSegmentation()
{
    if (cs_programID)
        glUniform1i(9, (segmentation) ? 1 : 0);
}

void RendererCore::setPlane()
{
    if (cs_programID)
        glUniform3f(8, plane_a, plane_b, plane_c);
}

void RendererCore::setMinVal()
{
    if(cs_programID)
    {
        if(datasize_bytes == 2)
            glUniform1i(2, min_val+1000);
        else
            glUniform1i(2, min_val);
    }
}

void RendererCore::setThreShold()
{
    if (cs_programID)
    {
        glUniform1f(7, threshold);
    }
}

void RendererCore::setMaxVal()
{
    if(cs_programID)
    {
        if(datasize_bytes == 2)
            glUniform1i(3, max_val+1000);
        else
            glUniform1i(3, max_val);
    }
}

void RendererCore::setMIP()
{
    if(cs_programID)
        glUniform1i(4, (use_mip) ? 1 : 0);
}

void RendererCore::setInitialCameraRotation()
{
    if(cs_programID)
    {
        main_cam.resetCamera();
        glUniform1i(5, (rotate_to_top) ? 1 : 0);
        glUniform1i(6, (rotate_to_bottom) ? 1 : 0);
    }
}

void RendererCore::setUniforms()
{
    if(cs_programID)
        glUniform3f(1, voxel_size.x, voxel_size.y, voxel_size.z);
    setAlpha();
    setMinVal();
    setMaxVal();
    setMIP();
    setInitialCameraRotation();

}

bool RendererCore::loadShader(std::string fn, bool reload)
{
    if(createShader(fn, reload))
    {
        if(createShaderProgram())
        {
            std::cout << "Shader Loaded Successfully!\n" << std::endl;
            // get Total Workgroup count
            int workgroup_size[3];
            glGetProgramiv(cs_programID, GL_COMPUTE_WORK_GROUP_SIZE, workgroup_size);
            workgroups_x = window_size.x / workgroup_size[0];
            workgroups_y = window_size.y / workgroup_size[1];
            glUseProgram(cs_programID);
            glUniform1f(0, alpha_scale);
            if(!loaded_dataset.empty())
            {
                setUniforms();
                main_cam.resetCamera();
            }
            return true;
        }
    }
    loaded_shader.clear();
    return false;
}

void RendererCore::render()
{
    GLuint64 elapsed_time = 0;
    GLuint query;
    glGenQueries(1, &query);

    if (main_cam.is_changed) {
		std::cout << "in render setupUBO\n";
        setupUBO(true);
    }

    glBindImageTexture(0, fbo_texID, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);

    glBeginQuery(GL_TIME_ELAPSED, query);
    glDispatchCompute(workgroups_x, workgroups_y, 1);
    glEndQuery(GL_TIME_ELAPSED);
    glGetQueryObjectui64v(query, GL_QUERY_RESULT, &elapsed_time);
    kerneltime_sum += (double) elapsed_time/1000000.0;

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);

    glBlitFramebuffer(0, 0, framebuffer_size.x, framebuffer_size.y,
                      0, 0, framebuffer_size.x, framebuffer_size.y,
                      GL_COLOR_BUFFER_BIT,
                      GL_LINEAR
                      );
}

void RendererCore::render2()
{
    timer.start("render");

    timer.start("RendererCode capture");
    const uint8_t* color_data = color_image.get_buffer();  // 获取图像数据指针
    timer.stop("RendererCode capture");


    timer.start("RendererCode transfrom");

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, color_image.get_width_pixels(), color_image.get_height_pixels(), 0, GL_BGRA, GL_UNSIGNED_BYTE, color_data);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, textureID);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
    timer.stop("RendererCode transfrom");

    //**********************************************************************************
    GLuint64 elapsed_time = 0;
    GLuint query;
    glGenQueries(1, &query);

    if (main_cam.is_changed) {
        std::cout << "in render setupUBO\n";
        setupUBO(true);
    }

    glBindImageTexture(0, fbo_texID, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);

    glBeginQuery(GL_TIME_ELAPSED, query);
    glDispatchCompute(workgroups_x, workgroups_y, 1);
    glEndQuery(GL_TIME_ELAPSED);
    glGetQueryObjectui64v(query, GL_QUERY_RESULT, &elapsed_time);
    kerneltime_sum += (double)elapsed_time / 1000000.0;

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);



    //*************************************************************************************
    //// 使用着色器程序
    //glBindFramebuffer(GL_FRAMEBUFFER, fbo_ID);
    ////// 渲染指令
    ////glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    ////glClear(GL_COLOR_BUFFER_BIT);
    //glUseProgram(shaderProgram);

    //// 设置线段颜色（红色）
    //GLint colorLocation = glGetUniformLocation(shaderProgram, "lineColor");
    //glUniform3f(colorLocation, 1.0f, 0.0f, 0.0f);

    //// 绑定 VAO，绘制线段
    //glBindVertexArray(VAO);


    //glDrawArrays(GL_LINES, 0, vertices.size() / 2);
    //glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_ID); // Bind FBO to read from
    //glReadBuffer(GL_COLOR_ATTACHMENT0);             // Read from Color Attachment 0
    //glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);      // Bind default framebuffer to draw to
    //glDrawBuffer(GL_BACK);                          // Draw to Back Buffer
    glBlitFramebuffer(0, 0, framebuffer_size.x, framebuffer_size.y,
        0, 0, framebuffer_size.x, framebuffer_size.y,
        GL_COLOR_BUFFER_BIT,
        GL_LINEAR
    );
    timer.stop("render");
}

void RendererCore::render_cube()
{
    timer.start("render");

    timer.start("RendererCode capture");
    const uint8_t* color_data = color_image.get_buffer();  // 获取图像数据指针
    timer.stop("RendererCode capture");


    timer.start("RendererCode transfrom");
    glUseProgram(cs_programID);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, color_image.get_width_pixels(), color_image.get_height_pixels(), 0, GL_BGRA, GL_UNSIGNED_BYTE, color_data);


    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
    timer.stop("RendererCode transfrom");

    //**********************************************************************************
    GLuint64 elapsed_time = 0;
    GLuint query;
    glGenQueries(1, &query);

    if (main_cam.is_changed) {
        std::cout << "in render setupUBO\n";
        setupUBO(true);
    }

    glBindImageTexture(0, fbo_texID, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);

    glBeginQuery(GL_TIME_ELAPSED, query);
    glDispatchCompute(workgroups_x, workgroups_y, 1);
    glEndQuery(GL_TIME_ELAPSED);
    glGetQueryObjectui64v(query, GL_QUERY_RESULT, &elapsed_time);
    kerneltime_sum += (double)elapsed_time / 1000000.0;

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);



    //*************************************************************************************
    //使用着色器程序
    if (if_detected_marker) {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_ID);
        glEnable(GL_DEPTH_TEST);
        glClear(GL_DEPTH_BUFFER_BIT); // also clear the depth buffer now!
        ourShader.use();
        glActiveTexture(GL_TEXTURE5);
        glBindTexture(GL_TEXTURE_2D, texture1);
        glActiveTexture(GL_TEXTURE6);
        glBindTexture(GL_TEXTURE_2D, texture2);
        unsigned int modelLoc = glGetUniformLocation(ourShader.ID, "model");
        unsigned int viewLoc = glGetUniformLocation(ourShader.ID, "view");
        // pass them to the shaders (3 different ways)
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, &view[0][0]);
        // note: currently we set the projection matrix each frame, but since the projection matrix rarely changes it's often best practice to set it outside the main loop only once.
        ourShader.setMat4("projection", projection);

        // render box
        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, 36);
    }

    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_ID); // Bind FBO to read from
    glReadBuffer(GL_COLOR_ATTACHMENT0);             // Read from Color Attachment 0
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);      // Bind default framebuffer to draw to
    glDrawBuffer(GL_BACK);                          // Draw to Back Buffer
    glBlitFramebuffer(0, 0, framebuffer_size.x, framebuffer_size.y,
        0, 0, framebuffer_size.x, framebuffer_size.y,
        GL_COLOR_BUFFER_BIT,
        GL_LINEAR
    );
    glUseProgram(cs_programID);
    timer.stop("render");
}

bool RendererCore::saveImage(std::string fn, std::string ext)
{
	//std::cout << "saveImage Framebuffer Size: " << framebuffer_size.x << ", " << framebuffer_size.y << std::endl;
    int stride = (framebuffer_size.x % 4) + (framebuffer_size.x * 3);
    unsigned char* img_data = new unsigned char[stride  * framebuffer_size.y * 3];
    // 输出img_data长度
	//std::cout << "saveImage img_data length: " << stride * framebuffer_size.y * 3 << std::endl;

    bool status = false;
    glReadPixels(0, 0, framebuffer_size.x, framebuffer_size.y, GL_RGB, GL_UNSIGNED_BYTE, img_data);
    stbi_flip_vertically_on_write(1);
    if(ext == ".png")
        status = stbi_write_png(fn.c_str(), framebuffer_size.x, framebuffer_size.y, 3, img_data, stride);
    else if(ext == ".jpg")
        status = stbi_write_jpg(fn.c_str(), framebuffer_size.x, framebuffer_size.y, 3, img_data, 100);
    else if(ext == ".bmp")
        status = stbi_write_bmp(fn.c_str(), framebuffer_size.x, framebuffer_size.y, 3, img_data);

    delete[] img_data;
    return status;
}

void RendererCore::getImageContent(){
    if (img_data_from_core == nullptr) {
        int stride = (framebuffer_size.x % 4) + (framebuffer_size.x * 3);
		// qbh modify from stride * framebuffer_size.y*3 to stride * framebuffer_size.y
        img_data_from_core = new unsigned char[stride * framebuffer_size.y];
    }
    glReadPixels(0, 0, framebuffer_size.x, framebuffer_size.y, GL_RGB, GL_UNSIGNED_BYTE, img_data_from_core);
}

void RendererCore::setupFBO()
{
    glGenFramebuffers(1,&fbo_ID);
    glBindFramebuffer(GL_FRAMEBUFFER,fbo_ID);

    glGenTextures(1, &fbo_texID);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, fbo_texID);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, framebuffer_size.x, framebuffer_size.y, 0, GL_RGBA, GL_FLOAT,0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	glBindTexture(GL_TEXTURE_2D, 0); //Unbind Texture

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_texID, 0); //Attach Texture to Framebuffer
    
    // Create a renderbuffer object for depth and stencil attachment
    GLuint rbo;
    glGenRenderbuffers(1, &rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, framebuffer_size.x, framebuffer_size.y);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);

	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER); //Check if Framebuffer is complete

    if(status != GL_FRAMEBUFFER_COMPLETE)
    {
        if(status == GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT)
            throw std::runtime_error("Framebuffer not complete. Error code: GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT");
        else if(status == GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT)
            throw std::runtime_error("Framebuffer not complete. Error code: GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT");
        else if(status == GL_FRAMEBUFFER_UNDEFINED)
            throw std::runtime_error("Framebuffer not complete. Error code: GL_FRAMEBUFFER_UNDEFINED");
        else if(status == GL_FRAMEBUFFER_UNSUPPORTED)
            throw std::runtime_error("Framebuffer not complete. Error code: GL_FRAMEBUFFER_UNSUPPORTED");
        else if(status == GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER)
            throw std::runtime_error("Framebuffer not complete. Error code: GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER");
        else if(status == GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER)
            throw std::runtime_error("Framebuffer not complete. Error code: GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER");
        else
            throw std::runtime_error("Framebuffer not complete.");
    }
}

void RendererCore::setupUBO(bool is_update)
{
    std::vector<float> cam_data;
    cam_data.clear();
    main_cam.setUBO(cam_data);
    bool is_init = false;
    if(!is_update || camera_ubo_ID == 0)
        is_init = true;

    if(is_init)
        glGenBuffers(1, &camera_ubo_ID);

    glBindBuffer(GL_UNIFORM_BUFFER, camera_ubo_ID);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(float)*cam_data.size(), cam_data.data(), GL_DYNAMIC_DRAW);

    if(is_init)
        glBindBufferBase(GL_UNIFORM_BUFFER, 1, camera_ubo_ID);

    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void RendererCore::readVolumeData(std::string fn)
{
    std::string ext = fn.substr(fn.length()-3, 3);
    void* volume_data = NULL;

    if(ext == "raw")
    {
        std::ifstream inf_file;
        inf_file.open(fn + ".inf");
        if(inf_file)
        {
            std::string line = "";
            voxel_size = glm::vec3(0,0,0);
            tex3D_dim = glm::ivec3(0,0,0);
            while(getline(inf_file, line))
            {
                // skip empty lines
                if(line.empty())
                    continue;
                else if(line == "#dimensions")
                {
                    getline(inf_file, line);
                    if(line.empty())
                    {
                        msg = "Dimensions for Volume Data not provided in \"raw.inf\" file.";
                        title = "Invalid .raw.inf file!";
                        return;
                    }
                    std::stringstream ss(line);
                    ss >> tex3D_dim.x;
                    ss >> tex3D_dim.y;
                    ss >> tex3D_dim.z;
                }
                else if(line == "#voxel-spacing")
                {
                    getline(inf_file, line);
                    if(line.empty())
                    {
                        msg = "Aspect Ratio for Volume Data not provided in \"raw.inf\" file.";
                        title = "Invalid .raw.inf file!";
                        return;
                    }
                    std::stringstream ss(line);
                    ss >> voxel_size.x;
                    ss >> voxel_size.y;
                    ss >> voxel_size.z;
                }
            }
            if(tex3D_dim == glm::ivec3(0,0,0))
            {
                msg = "Dimensions for Volume Data not provided in \"raw.inf\" file. Make sure the header is \"#dimesnsions\"";
                title = "Invalid .raw.inf file!";
                return;
            }

            if(voxel_size == glm::vec3(0,0,0))
            {
                msg = "Aspect Ratio for Volume Data not provided in \"raw.inf\" file. Make sure the header is \"#voxel-spacing\"";
                title = "Invalid .raw.inf file!";
                return;
            }
        }
        else
        {
            //If no .raw.inf file found write one, using user provided parameters.
            std::ofstream oinf_file;
            oinf_file.open(fn + ".inf");
            if(oinf_file)
            {
                oinf_file << "#dimensions\n" << tex3D_dim.x << " "
                     << tex3D_dim.y << " " << tex3D_dim.z << "\n\n"

                     << "#voxel-spacing\n" << voxel_size.x << " "
                     << voxel_size.y << " " << voxel_size.z << std::endl;
            }
        }
        std::ifstream raw_file;
        raw_file.open(fn, std::ios::binary);

        if(!raw_file)
        {
            msg = "Failed to Open RAW file...";
            title = "Error!";
            return;
        }
        int len = tex3D_dim.x * tex3D_dim.y * tex3D_dim.z;

        if(len == 0)
        {
            msg = "Texture Dimensions shouldn't contain any zeroes. Please provide a valid .raw.inf file.";
            title = "Invalid Data Size!";
            return;
        }

        //Read RAW file into byte array;
        if(datasize_bytes == 1)
            volume_data = (uint8_t*) new uint8_t[len]();
        else
            volume_data = (uint16_t*) new uint16_t[len]();
        raw_file.read((char*)volume_data, len * datasize_bytes);
    }
    else
    {
        unsigned int components = -1;
        glm::uvec3 dims(0, 0, 0);
        volume_data = readPVMvolume(fn.c_str(), &dims.x, &dims.y, &dims.z, &components, &voxel_size.x, &voxel_size.y, &voxel_size.z);
        tex3D_dim = glm::ivec3(dims.x, dims.y, dims.z);
        if(!volume_data)
        {
            msg = "Error reading PVM file";
            title = "Error!";
            return;
        }
    }

    std::cout << "Dataset dimensions: " << tex3D_dim.x << ", " << tex3D_dim.y << ", " << tex3D_dim.z << std::endl;
    std::cout << "Dataset Aspect ratio: " << voxel_size.x << ", " << voxel_size.y << ", " << voxel_size.z << std::endl;
	
    Constants::tex3D_dim = tex3D_dim;
    Constants::voxel_size = voxel_size;

    //Calculate Histogram
    int max_value = -1, min_value = 9000000, len = tex3D_dim.x * tex3D_dim.y * tex3D_dim.z;
    if(datasize_bytes == 2)
    {
        for(int i = 0; i < len; i++)
        {
            if(i == 8390640)
                continue;
           uint16_t val = (((uint16_t*)(volume_data))[i]);

            if(val > max_value)
                max_value = val;
            if(val < min_value)
                min_value = val;
        }
        max_val = max_value;
        max_dataset_val = max_value;
        min_val = min_value;
        min_dataset_val = min_value;
    }
    else
    {
        min_val = min_dataset_val = 0;
        max_val = max_dataset_val = 255;
    }

    for(int i = 0; i < len; i++)
    {
        uint16_t val = 0;
        if(datasize_bytes == 1)
            val = (((uint8_t*)(volume_data))[i]);
        else
        {
            val = (((uint16_t*)(volume_data))[i]);
            val = std::round(val * 255.0f/max_dataset_val);
        }
        if(val == 0)
            continue;
        histogram[val]++;

        if(histogram[val] > max_value)
            max_value = histogram[val];
    }

    for(int i = 0; i < histogram.size(); i++)
        histogram[i] = histogram[i] * 100.0f / max_value;

    //Upload data from array to 3D texture
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_3D, vol_tex3D);

    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    if(tex3D_dim.x % 4 != 0)
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage3D(GL_TEXTURE_3D, 0, (datasize_bytes == 1) ? GL_R8UI : GL_R16UI, tex3D_dim.x, tex3D_dim.y, tex3D_dim.z, 0, GL_RED_INTEGER, (datasize_bytes == 1) ? GL_UNSIGNED_BYTE : GL_UNSIGNED_SHORT, volume_data);
    if(ext == "pvm")
    {
        if(datasize_bytes == 1)
            free((uint8_t*)volume_data);
        else
            free((uint16_t*)volume_data);
    }
    else
    {
        if(datasize_bytes == 1)
            delete[] (uint8_t*)volume_data;
        else
            delete[] (uint16_t*)volume_data;
    }
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

    title = "File Loaded!";
    msg = "File Loaded Successfully!";

    if(!loaded_shader.empty())
    {
        setUniforms();
		main_cam.resetCamera(); // qbh for some reason I don't want to reset the camera
    }

    int idx = fn.find_last_of("/");
    loaded_dataset =  fn.substr(idx+1, fn.length() - idx);
}

bool RendererCore::createShader(std::string fn, bool reload)
{
    std::string shader_data = "";
    std::streamoff len;
    std::ifstream file;

    if(reload)
        fn = loaded_shader;

    file.open(fn, std::ios::binary);
    if(!file.is_open())
    {
        if(fn == "VolumeRenderer.cs")
            msg = "Failed to open default Shader file. Please select a shader file with the extension \".cs\" explicitly.";
        else
            msg = "Failed to open Shader file.";
        title = "Error!";
        return false;
    }

    file.seekg(0, std::ios::end);
    len = file.tellg();
    file.seekg(0, std::ios::beg);

    shader_data.resize(len);
    file.read(&shader_data[0], len);

    const GLchar* source = (const GLchar *) shader_data.c_str();

    cs_ID = glCreateShader(GL_COMPUTE_SHADER);
    glShaderSource(cs_ID, 1, &source,0);
    glCompileShader(cs_ID);

    GLint isCompiled = 0;
    glGetShaderiv(cs_ID, GL_COMPILE_STATUS, &isCompiled);
    if(isCompiled == GL_FALSE)
    {
        GLint max_length = 0;
        glGetShaderiv(cs_ID, GL_INFO_LOG_LENGTH, &max_length);
        std::vector<GLchar> infoLog(max_length);
        glGetShaderInfoLog(cs_ID, max_length, &max_length, &infoLog[0]);

        glDeleteShader(cs_ID);
        std::string err_log(infoLog.begin(), infoLog.end());
        std::cout << "\n" << err_log << std::endl;

        msg = "Failed to Compile Shader, detailed log is printed in console.";
        title = "Shader Error!";
        return false;
    }
    int idx = fn.find_last_of("/");
    loaded_shader = fn.substr(idx+1, fn.length() - idx);
    return true;
}

bool RendererCore::createShaderProgram()
{
    if(!cs_programID)
        cs_programID = glCreateProgram();
    glAttachShader(cs_programID, cs_ID);

    glLinkProgram(cs_programID);
    glDetachShader(cs_programID, cs_ID);
    glDeleteShader(cs_ID);

    GLint isLinked = 0;
    glGetProgramiv(cs_programID, GL_LINK_STATUS, &isLinked);
    if(isLinked == GL_FALSE)
    {
        GLint max_length = 0;
        glGetProgramiv(cs_programID, GL_INFO_LOG_LENGTH, &max_length);

        std::vector<GLchar> infoLog(max_length);
        glGetProgramInfoLog(cs_programID, max_length, &max_length, &infoLog[0]);

        glDeleteProgram(cs_programID);
        cs_programID = 0;

        std::string err_log(infoLog.begin(), infoLog.end());
        std::cout << "\n" << err_log << std::endl;

        msg = "Failed to Link Program Object, detailed log is printed in console.";
        title = "Shader Error!";
        return false;
    }

    title = "Shader Loaded!";
    msg = "Shader Loaded Successfully!";
    return true;
}

