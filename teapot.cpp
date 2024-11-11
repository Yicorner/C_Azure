#include "teapot.hpp"
// ���� GLM ��ѧ��





// ���徲̬����
namespace TeapotData {
    std::vector<Vertex> vertices;
    std::vector<GLuint> indices;
    const char* vertexShaderSource = R"(
#version 330 core
layout(location = 0) in vec3 aPos;    // ����λ��
layout(location = 1) in vec3 aNormal; // ������

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 FragPos;  // ���ݸ�Ƭ����ɫ���Ķ���λ��
out vec3 Normal;   // ���ݸ�Ƭ����ɫ���ķ�����

void main()
{
    FragPos = vec3(model * vec4(aPos, 1.0)); // ������������ϵ�µĶ���λ��
    Normal = mat3(transpose(inverse(model))) * aNormal; // ����任��ķ�����
    gl_Position = projection * view * vec4(FragPos, 1.0); // ����ü��ռ�����
}
)";

    const char* fragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;

in vec3 FragPos; // �Ӷ�����ɫ�����ݹ����Ķ���λ��
in vec3 Normal;  // �Ӷ�����ɫ�����ݹ����ķ�����

uniform vec3 lightPos;    // ��Դλ��
uniform vec3 viewPos;     // �۲���λ��
uniform vec3 objectColor; // ������ɫ

void main()
{
    // ������
    float ambientStrength = 0.2;
    vec3 ambient = ambientStrength * objectColor;

    // ������
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos); // ���߷���
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * objectColor;

    // ����߹�
    float specularStrength = 0.5;
    vec3 viewDir = normalize(viewPos - FragPos); // ���߷���
    vec3 reflectDir = reflect(-lightDir, norm);  // ���䷽��
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = specularStrength * spec * vec3(1.0);

    // �ϳɽ��
    vec3 result = ambient + diffuse + specular;
    FragColor = vec4(result, 1.0);
}
)";

    // �������
    const int patchdata_teapot[10][16] =
    {
        {  0,   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,  15, }, /* rim    */
        { 12,  13,  14,  15,  16,  17,  18,  19,  20,  21,  22,  23,  24,  25,  26,  27, }, /* body   */
        { 24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39, },
        { 40,  41,  42,  40,  43,  44,  45,  46,  47,  47,  47,  47,  48,  49,  50,  51, }, /* lid    */
        { 48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63, },
        { 64,  64,  64,  64,  65,  66,  67,  68,  69,  70,  71,  72,  39,  38,  37,  36, }, /* bottom */
        { 73,  74,  75,  76,  77,  78,  79,  80,  81,  82,  83,  84,  85,  86,  87,  88, }, /* handle */
        { 85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99, 100, },
        {101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, }, /* spout  */
        {113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128  }
    };

    const float cpdata_teapot[][3] =
    {
        { 1.40000f,  0.00000f,  2.40000f}, { 1.40000f, -0.78400f,  2.40000f},
        { 0.78400f, -1.40000f,  2.40000f}, { 0.00000f, -1.40000f,  2.40000f},
        { 1.33750f,  0.00000f,  2.53125f}, { 1.33750f, -0.74900f,  2.53125f},
        { 0.74900f, -1.33750f,  2.53125f}, { 0.00000f, -1.33750f,  2.53125f},
        { 1.43750f,  0.00000f,  2.53125f}, { 1.43750f, -0.80500f,  2.53125f},
        { 0.80500f, -1.43750f,  2.53125f}, { 0.00000f, -1.43750f,  2.53125f},
        { 1.50000f,  0.00000f,  2.40000f}, { 1.50000f, -0.84000f,  2.40000f},
        { 0.84000f, -1.50000f,  2.40000f}, { 0.00000f, -1.50000f,  2.40000f},
        { 1.75000f,  0.00000f,  1.87500f}, { 1.75000f, -0.98000f,  1.87500f},
        { 0.98000f, -1.75000f,  1.87500f}, { 0.00000f, -1.75000f,  1.87500f},
        { 2.00000f,  0.00000f,  1.35000f}, { 2.00000f, -1.12000f,  1.35000f},
        { 1.12000f, -2.00000f,  1.35000f}, { 0.00000f, -2.00000f,  1.35000f},
        { 2.00000f,  0.00000f,  0.90000f}, { 2.00000f, -1.12000f,  0.90000f},
        { 1.12000f, -2.00000f,  0.90000f}, { 0.00000f, -2.00000f,  0.90000f},
        { 2.00000f,  0.00000f,  0.45000f}, { 2.00000f, -1.12000f,  0.45000f},
        { 1.12000f, -2.00000f,  0.45000f}, { 0.00000f, -2.00000f,  0.45000f},
        { 1.50000f,  0.00000f,  0.22500f}, { 1.50000f, -0.84000f,  0.22500f},
        { 0.84000f, -1.50000f,  0.22500f}, { 0.00000f, -1.50000f,  0.22500f},
        { 1.50000f,  0.00000f,  0.15000f}, { 1.50000f, -0.84000f,  0.15000f},
        { 0.84000f, -1.50000f,  0.15000f}, { 0.00000f, -1.50000f,  0.15000f},
        { 0.00000f,  0.00000f,  3.15000f}, { 0.00000f, -0.00200f,  3.15000f},
        { 0.00200f,  0.00000f,  3.15000f}, { 0.80000f,  0.00000f,  3.15000f},
        { 0.80000f, -0.45000f,  3.15000f}, { 0.45000f, -0.80000f,  3.15000f},
        { 0.00000f, -0.80000f,  3.15000f}, { 0.00000f,  0.00000f,  2.85000f},
        { 0.20000f,  0.00000f,  2.70000f}, { 0.20000f, -0.11200f,  2.70000f},
        { 0.11200f, -0.20000f,  2.70000f}, { 0.00000f, -0.20000f,  2.70000f},
        { 0.40000f,  0.00000f,  2.55000f}, { 0.40000f, -0.22400f,  2.55000f},
        { 0.22400f, -0.40000f,  2.55000f}, { 0.00000f, -0.40000f,  2.55000f},
        { 1.30000f,  0.00000f,  2.55000f}, { 1.30000f, -0.72800f,  2.55000f},
        { 0.72800f, -1.30000f,  2.55000f}, { 0.00000f, -1.30000f,  2.55000f},
        { 1.30000f,  0.00000f,  2.40000f}, { 1.30000f, -0.72800f,  2.40000f},
        { 0.72800f, -1.30000f,  2.40000f}, { 0.00000f, -1.30000f,  2.40000f},
        { 0.00000f,  0.00000f,  0.00000f}, { 0.00000f, -1.42500f,  0.00000f},
        { 0.79800f, -1.42500f,  0.00000f}, { 1.42500f, -0.79800f,  0.00000f},
        { 1.42500f,  0.00000f,  0.00000f}, { 0.00000f, -1.50000f,  0.07500f},
        { 0.84000f, -1.50000f,  0.07500f}, { 1.50000f, -0.84000f,  0.07500f},
        { 1.50000f,  0.00000f,  0.07500f}, {-1.60000f,  0.00000f,  2.02500f},
        {-1.60000f, -0.30000f,  2.02500f}, {-1.50000f, -0.30000f,  2.25000f},
        {-1.50000f,  0.00000f,  2.25000f}, {-2.30000f,  0.00000f,  2.02500f},
        {-2.30000f, -0.30000f,  2.02500f}, {-2.50000f, -0.30000f,  2.25000f},
        {-2.50000f,  0.00000f,  2.25000f}, {-2.70000f,  0.00000f,  2.02500f},
        {-2.70000f, -0.30000f,  2.02500f}, {-3.00000f, -0.30000f,  2.25000f},
        {-3.00000f,  0.00000f,  2.25000f}, {-2.70000f,  0.00000f,  1.80000f},
        {-2.70000f, -0.30000f,  1.80000f}, {-3.00000f, -0.30000f,  1.80000f},
        {-3.00000f,  0.00000f,  1.80000f}, {-2.70000f,  0.00000f,  1.57500f},
        {-2.70000f, -0.30000f,  1.57500f}, {-3.00000f, -0.30000f,  1.35000f},
        {-3.00000f,  0.00000f,  1.35000f}, {-2.50000f,  0.00000f,  1.12500f},
        {-2.50000f, -0.30000f,  1.12500f}, {-2.65000f, -0.30000f,  0.93750f},
        {-2.65000f,  0.00000f,  0.93750f}, {-2.00000f,  0.00000f,  0.90000f},
        {-2.00000f, -0.30000f,  0.90000f}, {-1.90000f, -0.30000f,  0.60000f},
        {-1.90000f,  0.00000f,  0.60000f}, { 1.70000f,  0.00000f,  1.42500f},
        { 1.70000f, -0.66000f,  1.42500f}, { 1.70000f, -0.66000f,  0.60000f},
        { 1.70000f,  0.00000f,  0.60000f}, { 2.60000f,  0.00000f,  1.42500f},
        { 2.60000f, -0.66000f,  1.42500f}, { 3.10000f, -0.66000f,  0.82500f},
        { 3.10000f,  0.00000f,  0.82500f}, { 2.30000f,  0.00000f,  2.10000f},
        { 2.30000f, -0.25000f,  2.10000f}, { 2.40000f, -0.25000f,  2.02500f},
        { 2.40000f,  0.00000f,  2.02500f}, { 2.70000f,  0.00000f,  2.40000f},
        { 2.70000f, -0.25000f,  2.40000f}, { 3.30000f, -0.25000f,  2.40000f},
        { 3.30000f,  0.00000f,  2.40000f}, { 2.80000f,  0.00000f,  2.47500f},
        { 2.80000f, -0.25000f,  2.47500f}, { 3.52500f, -0.25000f,  2.49375f},
        { 3.52500f,  0.00000f,  2.49375f}, { 2.90000f,  0.00000f,  2.47500f},
        { 2.90000f, -0.15000f,  2.47500f}, { 3.45000f, -0.15000f,  2.51250f},
        { 3.45000f,  0.00000f,  2.51250f}, { 2.80000f,  0.00000f,  2.40000f},
        { 2.80000f, -0.15000f,  2.40000f}, { 3.20000f, -0.15000f,  2.40000f},
        { 3.20000f,  0.00000f,  2.40000f}
    };
}

void generateTeapotData() {
	using namespace TeapotData;
    // ʹ�� vertexShaderSource, fragmentShaderSource, patchdata_teapot �� cpdata_teapot
    // ��� TeapotData::vertices �� TeapotData::indices ���߼�
    const int tessellation = 10; // ÿ����Ƭ��ϸ�ִ���

    for (int patchNum = 0; patchNum < 10; ++patchNum)
    {
        // ��ȡ��ǰ��Ƭ�Ŀ��Ƶ�����
        int patchIndices[16];
        for (int i = 0; i < 16; ++i)
        {
            patchIndices[i] = patchdata_teapot[patchNum][i];
        }

        // �������Ƶ�����
        glm::vec3 controlPoints[4][4];
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                int index = patchIndices[i * 4 + j];
                controlPoints[i][j] = glm::vec3(
                    cpdata_teapot[index][0],
                    cpdata_teapot[index][1],
                    cpdata_teapot[index][2]
                );
            }
        }

        // ����ע�ͽ��б任
        // ���� Rim, Body, Lid, Bottom����Ҫ��ת�ĸ�����
        // ���� Handle �� Spout����Ҫ�� x-y ƽ���Ϸ�ת z ��

        int type = 0; // 0: Rim, Body, Lid, Bottom; 1: Handle, Spout
        if (patchNum < 6)
            type = 0; // Rim, Body, Lid, Bottom
        else
            type = 1; // Handle, Spout

        if (type == 0)
        {
            // ��ת�ĸ�����
            for (int rotate = 0; rotate < 4; ++rotate)
            {
                // ������ת����
                float angle = glm::radians(rotate * 90.0f);
                glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0, 0, 1));

                // ��¼��ǰ�������ʼ����
                int vertexOffset = TeapotData::vertices.size();

                // ���ɶ���ͷ�����
                for (int i = 0; i <= tessellation; ++i)
                {
                    float u = (float)i / tessellation;
                    for (int j = 0; j <= tessellation; ++j)
                    {
                        float v = (float)j / tessellation;

                        // ����λ�úͷ�����
                        glm::vec3 position = evaluateBezierPatch(u, v, controlPoints);
                        glm::vec3 du = evaluateBezierPatchDu(u, v, controlPoints);
                        glm::vec3 dv = evaluateBezierPatchDv(u, v, controlPoints);
                        glm::vec3 normal = glm::normalize(glm::cross(du, dv));

                        // Ӧ����ת
                        position = glm::vec3(rotation * glm::vec4(position, 1.0f));
                        normal = glm::normalize(glm::vec3(rotation * glm::vec4(normal, 0.0f)));

                        // �洢����
                        Vertex vertex;
                        vertex.position = position;
                        vertex.normal = normal;
                        TeapotData::vertices.push_back(vertex);

                        // �洢����
                        if (i < tessellation && j < tessellation)
                        {
                            int current = vertexOffset + i * (tessellation + 1) + j;
                            int next = vertexOffset + (i + 1) * (tessellation + 1) + j;

                            TeapotData::indices.push_back(current);
                            TeapotData::indices.push_back(next);
                            TeapotData::indices.push_back(current + 1);

                            TeapotData::indices.push_back(current + 1);
                            TeapotData::indices.push_back(next);
                            TeapotData::indices.push_back(next + 1);
                        }
                    }
                }
            }
        }
        else
        {
            // ԭʼ��Ƭ
            {
                // ��¼��ǰ�������ʼ����
                int vertexOffset = TeapotData::vertices.size();

                // ���ɶ���ͷ�����
                for (int i = 0; i <= tessellation; ++i)
                {
                    float u = (float)i / tessellation;
                    for (int j = 0; j <= tessellation; ++j)
                    {
                        float v = (float)j / tessellation;

                        // ����λ�úͷ�����
                        glm::vec3 position = evaluateBezierPatch(u, v, controlPoints);
                        glm::vec3 du = evaluateBezierPatchDu(u, v, controlPoints);
                        glm::vec3 dv = evaluateBezierPatchDv(u, v, controlPoints);
                        glm::vec3 normal = glm::normalize(glm::cross(du, dv));

                        // �洢����
                        Vertex vertex;
                        vertex.position = position;
                        vertex.normal = normal;
                        TeapotData::vertices.push_back(vertex);

                        // �洢����
                        if (i < tessellation && j < tessellation)
                        {
                            int current = vertexOffset + i * (tessellation + 1) + j;
                            int next = vertexOffset + (i + 1) * (tessellation + 1) + j;

                            TeapotData::indices.push_back(current);
                            TeapotData::indices.push_back(next);
                            TeapotData::indices.push_back(current + 1);

                            TeapotData::indices.push_back(current + 1);
                            TeapotData::indices.push_back(next);
                            TeapotData::indices.push_back(next + 1);
                        }
                    }
                }
            }

            // ��ת z �����Ƭ
            {
                // ��¼��ǰ�������ʼ����
                int vertexOffset = TeapotData::vertices.size();

                // ���ɶ���ͷ�����
                for (int i = 0; i <= tessellation; ++i)
                {
                    float u = (float)i / tessellation;
                    for (int j = 0; j <= tessellation; ++j)
                    {
                        float v = (float)j / tessellation;

                        // ����λ�úͷ�����
                        glm::vec3 position = evaluateBezierPatch(u, v, controlPoints);
                        glm::vec3 du = evaluateBezierPatchDu(u, v, controlPoints);
                        glm::vec3 dv = evaluateBezierPatchDv(u, v, controlPoints);
                        glm::vec3 normal = glm::normalize(glm::cross(du, dv));

                        // ��ת z ��
                        position.y = -position.y;
                        normal.y = -normal.y;

                        // �洢����
                        Vertex vertex;
                        vertex.position = position;
                        vertex.normal = normal;
                        TeapotData::vertices.push_back(vertex);

                        // �洢����
                        if (i < tessellation && j < tessellation)
                        {
                            int current = vertexOffset + i * (tessellation + 1) + j;
                            int next = vertexOffset + (i + 1) * (tessellation + 1) + j;

                            TeapotData::indices.push_back(current);
                            TeapotData::indices.push_back(current + 1);
                            TeapotData::indices.push_back(next);

                            TeapotData::indices.push_back(current + 1);
                            TeapotData::indices.push_back(next + 1);
                            TeapotData::indices.push_back(next);
                        }
                    }
                }
            }
        }
    }
}

// ���� Bezier ������
void computeBezierBasis(float t, float& B0, float& B1, float& B2, float& B3)
{
    float u = t;
    float oneMinusU = 1.0f - u;

    B0 = oneMinusU * oneMinusU * oneMinusU;
    B1 = 3 * u * oneMinusU * oneMinusU;
    B2 = 3 * u * u * oneMinusU;
    B3 = u * u * u;
}

// ���� Bezier ��������һ�׵���
void computeBezierBasisDerivatives(float t, float& dB0, float& dB1, float& dB2, float& dB3)
{
    float u = t;
    float oneMinusU = 1.0f - u;

    dB0 = -3 * oneMinusU * oneMinusU;
    dB1 = 3 * oneMinusU * oneMinusU - 6 * u * oneMinusU;
    dB2 = 6 * u * oneMinusU - 3 * u * u;
    dB3 = 3 * u * u;
}

// ���� Bezier ����
glm::vec3 evaluateBezierPatch(float u, float v, const glm::vec3 controlPoints[4][4])
{
    float Bu[4], Bv[4];
    computeBezierBasis(u, Bu[0], Bu[1], Bu[2], Bu[3]);
    computeBezierBasis(v, Bv[0], Bv[1], Bv[2], Bv[3]);

    glm::vec3 point(0.0f);

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            point += controlPoints[i][j] * Bu[i] * Bv[j];
        }
    }
    return point;
}

// ���� Bezier ����� u ��ƫ����
glm::vec3 evaluateBezierPatchDu(float u, float v, const glm::vec3 controlPoints[4][4])
{
    float Bu[4], Bv[4];
    float dBu[4], dBv[4];

    computeBezierBasis(u, Bu[0], Bu[1], Bu[2], Bu[3]);
    computeBezierBasis(v, Bv[0], Bv[1], Bv[2], Bv[3]);
    computeBezierBasisDerivatives(u, dBu[0], dBu[1], dBu[2], dBu[3]);

    glm::vec3 du(0.0f);

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            du += controlPoints[i][j] * dBu[i] * Bv[j];
        }
    }
    return du;
}

// ���� Bezier ����� v ��ƫ����
glm::vec3 evaluateBezierPatchDv(float u, float v, const glm::vec3 controlPoints[4][4])
{
    float Bu[4], Bv[4];
    float dBu[4], dBv[4];

    computeBezierBasis(u, Bu[0], Bu[1], Bu[2], Bu[3]);
    computeBezierBasis(v, Bv[0], Bv[1], Bv[2], Bv[3]);
    computeBezierBasisDerivatives(v, dBv[0], dBv[1], dBv[2], dBv[3]);

    glm::vec3 dv(0.0f);

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            dv += controlPoints[i][j] * Bu[i] * dBv[j];
        }
    }
    return dv;
}