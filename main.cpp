#include <conio.h>  // Windows下用于非阻塞式获取键盘输入
#include <chrono>
#include <filesystem>
#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <fstream>  // 需要包含这个头文件
#include <locale>
#include <codecvt>

#include "GetCords.hpp"
#include "realTimeDisplay.hpp"
#include "getSample.hpp"
#include "work.hpp"
#include "AllEnum.hpp"
#include "constants.hpp"

void change_device_config(k4a::device& device, k4a_device_configuration_t& config) {
    device = k4a::device::open(0);
	// Configure the device
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_1080P;   // 1920x1080
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;       // 640x576
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.synchronized_images_only = true;                 // Ensure synchronized captures
	config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
}


int main()
{ 
    std::ofstream error_file("error_log.txt");
    std::ofstream file("normal_log.txt", std::ios::out | std::ios::binary);
    std::streambuf* original_cout = std::cout.rdbuf();
    if (Constants::if_log_file == 1) {
        std::cerr.rdbuf(error_file.rdbuf());

        // 将 std::cout 重定向到文件
        std::cout.rdbuf(file.rdbuf());
    }



    // Open Kinect device
    k4a::device device;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	change_device_config(device, config);
    device.start_cameras(&config);

	// realtimedisplay thread
	RealTimeDisplayState State = Constants::real_time_display_state;
	RealTimeDisplay realtimedisplay;
    //std::thread realTimeDisplay_thread(std::bind(&RealTimeDisplay::realTimeDisplay, &realtimedisplay, std::ref(device), std::ref(State)));

	// Infact, I don't need the object of getsample, just take it as an meanningless argumrnt and omit it.
    GetSample getsample;
	Work work(device, config);

    if (Constants::if_multi_thread == 1) {
        work.run_multi_thread(getsample);
    }
    else if(Constants::if_multi_thread == 2){
		work.run_multi_thread2(getsample);
    }
    else {
        work.run(getsample);

    }

	//realTimeDisplay_thread.join();
    device.stop_cameras();
    device.close();


    std::cout.rdbuf(original_cout);
    file.close();
    error_file.close();
    return 0;
}

// 单纯画茶壶
//#include <glad/glad.h>
//#include <GLFW/glfw3.h>
//
//#include <iostream>
//#include <vector>
//#include <cmath>
//
//#include "teapot.hpp"
//#define WIDTH 1920
//#define HEIGHT 1080
//using namespace TeapotData;
//
//
//int main()
//{
//    // 初始化 GLFW
//    glfwInit();
//    // 配置 GLFW
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
//    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//
//    // 创建窗口
//    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "Teapot", NULL, NULL);
//    if (window == NULL)
//    {
//        std::cout << "Failed to create GLFW window\n";
//        glfwTerminate();
//        return -1;
//    }
//    glfwMakeContextCurrent(window);
//
//    // 加载 OpenGL 函数指针
//    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
//    {
//        std::cout << "Failed to initialize GLAD\n";
//        return -1;
//    }
//
//    // 设置视口
//    glViewport(0, 0, WIDTH, HEIGHT);
//
//    // 启用深度测试
//    glEnable(GL_DEPTH_TEST);
//
//    // 编译顶点着色器
//    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
//    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
//    glCompileShader(vertexShader);
//    // 检查编译错误
//    int success;
//    char infoLog[512];
//    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
//    if (!success)
//    {
//        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
//        std::cout << "ERROR::VERTEX_SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
//    }
//
//    // 编译片段着色器
//    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
//    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
//    glCompileShader(fragmentShader);
//    // 检查编译错误
//    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
//    if (!success)
//    {
//        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
//        std::cout << "ERROR::FRAGMENT_SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
//    }
//
//    // 链接着色器程序
//    GLuint shaderProgram = glCreateProgram();
//    glAttachShader(shaderProgram, vertexShader);
//    glAttachShader(shaderProgram, fragmentShader);
//    glLinkProgram(shaderProgram);
//    // 检查链接错误
//    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
//    if (!success)
//    {
//        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
//        std::cout << "ERROR::SHADER_PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
//    }
//    // 删除着色器
//    glDeleteShader(vertexShader);
//    glDeleteShader(fragmentShader);
//
//	generateTeapotData();
//
//    // 创建 VAO、VBO、EBO
//    GLuint VAO, VBO, EBO;
//
//    glGenVertexArrays(1, &VAO);
//    glGenBuffers(1, &VBO);
//    glGenBuffers(1, &EBO);
//
//    // 绑定 VAO
//    glBindVertexArray(VAO);
//
//    // 绑定并填充 VBO
//    glBindBuffer(GL_ARRAY_BUFFER, VBO);
//    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);
//
//    // 绑定并填充 EBO
//    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
//    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);
//
//    // 设置顶点属性指针
//    // 位置属性
//    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
//    glEnableVertexAttribArray(0);
//    // 法向量属性
//    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
//    glEnableVertexAttribArray(1);
//
//    // 解绑 VAO
//    glBindVertexArray(0);
//
//    // 定义观察者位置
//    glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 6.0f);
//
//    // 渲染循环
//    while (!glfwWindowShouldClose(window))
//    {
//
//        // 清空颜色和深度缓冲
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//        // 使用着色器程序
//        glUseProgram(shaderProgram);
//
//        // 设置变换矩阵
//        glm::mat4 model = glm::mat4(1.0f);
//
//        // 应用旋转矩阵
//        float angle = (float)glfwGetTime() * glm::radians(20.0f); // 每秒旋转 20 度
//        glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0.0f, 1.0f, 0.0f));
//
//        model = rotationMatrix * model;
//
//        // 设置 uniform 变量
//        GLint modelLoc = glGetUniformLocation(shaderProgram, "model");
//        GLint viewLoc = glGetUniformLocation(shaderProgram, "view");
//        GLint projLoc = glGetUniformLocation(shaderProgram, "projection");
//
//        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
//
//        glm::mat4 view = glm::translate(glm::mat4(1.0f), -cameraPos);
//        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
//
//        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)WIDTH / HEIGHT, 0.1f, 100.0f);
//        glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
//
//        // 设置光照和颜色
//        glUniform3f(glGetUniformLocation(shaderProgram, "lightPos"), 0.0f, 5.0f, 5.0f);
//        glUniform3f(glGetUniformLocation(shaderProgram, "viewPos"), cameraPos.x, cameraPos.y, cameraPos.z);
//        glUniform3f(glGetUniformLocation(shaderProgram, "objectColor"), 0.6f, 0.3f, 0.1f);
//
//        // 绘制茶壶
//        glBindVertexArray(VAO);
//        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
//        glBindVertexArray(0);
//
//        // 交换缓冲区和查询事件
//        glfwSwapBuffers(window);
//        glfwPollEvents();
//    }
//
//    // 释放资源
//    glDeleteVertexArrays(1, &VAO);
//    glDeleteBuffers(1, &VBO);
//    glDeleteBuffers(1, &EBO);
//
//    glfwTerminate();
//    return 0;
//}