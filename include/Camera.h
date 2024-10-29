#ifndef CAMERA_H
#define CAMERA_H

#include "glm/mat4x4.hpp"
#include "glm/vec4.hpp"
#include "AllEnum.hpp"

#include <vector>

class Camera
{
    public:
        Camera();
        Camera(float y_FOV, float rot_speed = 0.7f, float mov_speed = 0.3f);
         ~Camera();

		void moveCamera(BodyLocation bodylocation, std::vector<std::vector<float>> body3Dlocation_list);
        void resetCamera();
        void setOrientation(float zoom, float zenith, float azimuth);
        void setViewMatrix(glm::vec4 eye, glm::vec4 side, glm::vec4 up, glm::vec4 look_at);
        void setUBO(std::vector<float>& cam_data);
        bool is_changed;
        glm::vec4 look_at;
        glm::vec4 side;
        glm::vec4 up;
        glm::vec4 eye;
        glm::mat4 rot_mat;
        float rotation_speed; // qbh
        glm::mat4 transform_depthCamera2fakeman(glm::mat3 depthCameraColorCord, glm::mat3 NDCfakemanCord);
		float cal_vec3dist(glm::vec3 a, glm::vec3 b);
    private:
        float view_plane_dist, y_FOV,mov_speed, zenith, azimuth, radius, tot_zenith, tot_azimuth, tot2_azimuth;
        glm::mat4 view2world_mat;



};

#endif // CAMERA_H
