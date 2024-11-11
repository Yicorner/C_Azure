#ifndef TEAPOT_HPP
#define TEAPOT_HPP


#include <iostream>
#include <vector>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glad/glad.h>

// ����ṹ��
struct Vertex
{
    glm::vec3 position;
    glm::vec3 normal;
};

// �����ռ� TeapotData�����ڷ�װ����
namespace TeapotData {
    extern std::vector<Vertex> vertices;
    extern std::vector<GLuint> indices;
    extern const char* vertexShaderSource;
    extern const char* fragmentShaderSource;
    // ������ά���������
    extern const int patchdata_teapot[10][16];
    extern const float cpdata_teapot[][3];
}
// ���������������ɲ�����ݵĺ�����
void generateTeapotData();
// ���� Bezier ������
void computeBezierBasis(float t, float& B0, float& B1, float& B2, float& B3);

// ���� Bezier ��������һ�׵���
void computeBezierBasisDerivatives(float t, float& dB0, float& dB1, float& dB2, float& dB3);

// ���� Bezier ����
glm::vec3 evaluateBezierPatch(float u, float v, const glm::vec3 controlPoints[4][4]);

// ���� Bezier ����� u ��ƫ����
glm::vec3 evaluateBezierPatchDu(float u, float v, const glm::vec3 controlPoints[4][4]);

// ���� Bezier ����� v ��ƫ����
glm::vec3 evaluateBezierPatchDv(float u, float v, const glm::vec3 controlPoints[4][4]);
#endif // TEAPOT_HPP