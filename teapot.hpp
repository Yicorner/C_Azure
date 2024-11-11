#ifndef TEAPOT_HPP
#define TEAPOT_HPP


#include <iostream>
#include <vector>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glad/glad.h>

// 顶点结构体
struct Vertex
{
    glm::vec3 position;
    glm::vec3 normal;
};

// 命名空间 TeapotData，用于封装数据
namespace TeapotData {
    extern std::vector<Vertex> vertices;
    extern std::vector<GLuint> indices;
    extern const char* vertexShaderSource;
    extern const char* fragmentShaderSource;
    // 声明多维数组的类型
    extern const int patchdata_teapot[10][16];
    extern const float cpdata_teapot[][3];
}
// 函数声明（如生成茶壶数据的函数）
void generateTeapotData();
// 计算 Bezier 基函数
void computeBezierBasis(float t, float& B0, float& B1, float& B2, float& B3);

// 计算 Bezier 基函数的一阶导数
void computeBezierBasisDerivatives(float t, float& dB0, float& dB1, float& dB2, float& dB3);

// 评估 Bezier 曲面
glm::vec3 evaluateBezierPatch(float u, float v, const glm::vec3 controlPoints[4][4]);

// 计算 Bezier 曲面对 u 的偏导数
glm::vec3 evaluateBezierPatchDu(float u, float v, const glm::vec3 controlPoints[4][4]);

// 计算 Bezier 曲面对 v 的偏导数
glm::vec3 evaluateBezierPatchDv(float u, float v, const glm::vec3 controlPoints[4][4]);
#endif // TEAPOT_HPP