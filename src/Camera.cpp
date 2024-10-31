#include "Camera.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/matrix_access.hpp"
#include "glm/gtc/constants.hpp"

#include <iostream>
#include <algorithm>
#include <math.h>
#include <Eigen/dense>

Camera::Camera()
{
    //ctor
}

Camera::Camera(float y_FOV, float rot_speed, float mov_speed) : y_FOV(y_FOV), rotation_speed(rot_speed), mov_speed(mov_speed)
{
    // y_FOV is 2 times
    view_plane_dist =  1/tan(y_FOV * glm::pi<float>()/360);
    is_changed = true;
    resetCamera();
    //ctor
}

Camera::~Camera()
{

}

void Camera::resetCamera()
{
    view2world_mat = glm::mat4(1.0f);
    rot_mat = glm::mat4(1.0f);
    setViewMatrix(glm::vec4(0, 0, 3, 1),
                   glm::vec4(1, 0, 0, 0),
                   glm::vec4(0, 1, 0, 0),
                   glm::vec4(0, 0, -1, 0)
                   );
    zenith = glm::pi<float>()/2.0;
    azimuth = 0;
    tot_zenith = 0;
    tot2_azimuth = tot_azimuth =0;
    radius = 3;
}

void Camera::moveCamera(BodyLocation bodylocation, std::vector<std::vector<float>> body3Dlocation_list)
{
    glm::mat3 fakemanchest(
        -0.211112f, -0.25f, 0.161112f, // 第一行是一个点
        -0.281456f, -0.227778f, 0.11111f, // 第二行是一个点
        -0.177778f, -0.244444f, 0.047222f      // 第三行是一个点
    );
    glm::mat3 fakemanhead(
        -0.168519f, -0.151852f, 0.677778f,  // 第一行是一个点
        -0.137037f, -0.225926f, 0.659259f, // 第二行是一个点
        -0.135185f, -0.166667f, 0.601852f     // 第三行是一个点
    );
    glm::mat3 fakemanabdo(
        -0.101235f, -0.180247f, -0.269136f,  // 第一行是一个点
        -0.195062f, -0.150617f, -0.276543f, // 第二行是一个点
        -0.103704f, -0.177778f, -0.375309f     // 第三行是一个点
    );
    //glm::mat3 depthCameraImage2chest(
    //    11.4616f, 3.58519f, 134.144f,  // 第一行是一个点
    //    -24.4033f, -35.7827f, 150.218f,  // 第二行是一个点
    //    -56.8212f, 25.2224f, 142.874f     // 第三行是一个点
    //);
    //glm::mat3 depthCameraImage10head(
    //    -35.4482f, 24.9432f, 192.193f,  // 第一行是一个点
    //    -64.1256f, -7.67211f, 159.326f,  // 第二行是一个点
    //    -13.3063f, -8.67443f, 156.485f     // 第三行是一个点
    //);
	glm::mat3 depthCamera(
        body3Dlocation_list[0][0], body3Dlocation_list[0][1], body3Dlocation_list[0][2],  // 第一行是一个点
		body3Dlocation_list[1][0], body3Dlocation_list[1][1], body3Dlocation_list[1][2],  // 第二行是一个点
		body3Dlocation_list[2][0], body3Dlocation_list[2][1], body3Dlocation_list[2][2]     // 第三行是一个点
	);
    glm::mat4 camera2fakemanMat;
    if (bodylocation == BodyLocation::CHEST)
    {
        camera2fakemanMat = transform_depthCamera2fakeman(depthCamera, fakemanchest);
	}
	else if (bodylocation == BodyLocation::HEAD)
	{
        camera2fakemanMat = transform_depthCamera2fakeman(depthCamera, fakemanhead);
	}
	else
	{
        camera2fakemanMat = transform_depthCamera2fakeman(depthCamera, fakemanabdo);
	}
    camera2fakemanMat = glm::transpose(camera2fakemanMat);
    setViewMatrix(glm::vec4(camera2fakemanMat[3]), glm::vec4(camera2fakemanMat[0]), -glm::vec4(camera2fakemanMat[1]), glm::vec4(camera2fakemanMat[2]));
}

float Camera::cal_vec3dist(glm::vec3 a, glm::vec3 b)
{
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

glm::mat4 Camera::transform_depthCamera2fakeman(glm::mat3 depthCameraColorCord, glm::mat3 NDCfakemanCord)
{
 //   //print glm::vec3(NDCfakemanCord[0]
	//std::cout << "NDCfakemanCord[0] : " << glm::vec3(NDCfakemanCord[0]).x << " " << glm::vec3(NDCfakemanCord[0]).y << " " << glm::vec3(NDCfakemanCord[0]).z << std::endl;
	//std::cout << "NDCfakemanCord[1] : " << glm::vec3(NDCfakemanCord[1]).x << " " << glm::vec3(NDCfakemanCord[1]).y << " " << glm::vec3(NDCfakemanCord[1]).z << std::endl;
	//std::cout << "NDCfakemanCord[2] : " << glm::vec3(NDCfakemanCord[2]).x << " " << glm::vec3(NDCfakemanCord[2]).y << " " << glm::vec3(NDCfakemanCord[2]).z << std::endl;
 //   // print glm::vec3(depthCameraColorCord[0]
	//std::cout << "depthCameraColorCord[0] : " << glm::vec3(depthCameraColorCord[0]).x << " " << glm::vec3(depthCameraColorCord[0]).y << " " << glm::vec3(depthCameraColorCord[0]).z << std::endl;
	//std::cout << "depthCameraColorCord[1] : " << glm::vec3(depthCameraColorCord[1]).x << " " << glm::vec3(depthCameraColorCord[1]).y << " " << glm::vec3(depthCameraColorCord[1]).z << std::endl;
	//std::cout << "depthCameraColorCord[2] : " << glm::vec3(depthCameraColorCord[2]).x << " " << glm::vec3(depthCameraColorCord[2]).y << " " << glm::vec3(depthCameraColorCord[2]).z << std::endl;


    float cameraDist1 = cal_vec3dist(glm::vec3(depthCameraColorCord[0]), glm::vec3(depthCameraColorCord[1]));
    float cameraDist2 = cal_vec3dist(glm::vec3(depthCameraColorCord[1]), glm::vec3(depthCameraColorCord[2]));
    float cameraDist3 = cal_vec3dist(glm::vec3(depthCameraColorCord[0]), glm::vec3(depthCameraColorCord[2]));

    float NDCDist1 = cal_vec3dist(glm::vec3(NDCfakemanCord[0]), glm::vec3(NDCfakemanCord[1]));
    float NDCDist2 = cal_vec3dist(glm::vec3(NDCfakemanCord[1]), glm::vec3(NDCfakemanCord[2]));
    float NDCDist3 = cal_vec3dist(glm::vec3(NDCfakemanCord[0]), glm::vec3(NDCfakemanCord[2]));


	
    float scaleCamera2fakeman = (cameraDist1 / NDCDist1 + cameraDist2 / NDCDist2 + cameraDist3 / NDCDist3) / 3;
    // norm them into opengl world cordinates
	glm::mat3 normedCameraColorCords = depthCameraColorCord / scaleCamera2fakeman;
	/*std::cout << "normedCameraColorCords[0] : " << glm::vec3(normedCameraColorCords[0]).x << " " << glm::vec3(normedCameraColorCords[0]).y << " " << glm::vec3(normedCameraColorCords[0]).z << std::endl;
	std::cout << "normedCameraColorCords[1] : " << glm::vec3(normedCameraColorCords[1]).x << " " << glm::vec3(normedCameraColorCords[1]).y << " " << glm::vec3(normedCameraColorCords[1]).z << std::endl;
	std::cout << "normedCameraColorCords[2] : " << glm::vec3(normedCameraColorCords[2]).x << " " << glm::vec3(normedCameraColorCords[2]).y << " " << glm::vec3(normedCameraColorCords[2]).z << std::endl;*/
	
	// move the camera to proper origin
    glm::vec3 centroid_normed = (normedCameraColorCords[0] + normedCameraColorCords[1] + normedCameraColorCords[2]) / 3.0f;
    glm::vec3 centroid_NDC = (NDCfakemanCord[0] + NDCfakemanCord[1] + NDCfakemanCord[2]) / 3.0f;
    
    glm::vec3 normed_centered[3];
    glm::vec3 NDC_centered[3];

    for (int i = 0; i < 3; ++i) {
        normed_centered[i] = normedCameraColorCords[i] - centroid_normed;
        NDC_centered[i] = NDCfakemanCord[i] - centroid_NDC;
    }
    // Convert GLM vectors to Eigen vectors
    Eigen::MatrixXd normed_matrix(3, 3);
    Eigen::MatrixXd NDC_matrix(3, 3);

    for (int i = 0; i < 3; ++i) {
        normed_matrix(i, 0) = normed_centered[i].x;
        normed_matrix(i, 1) = normed_centered[i].y;
        normed_matrix(i, 2) = normed_centered[i].z;

        NDC_matrix(i, 0) = NDC_centered[i].x;
        NDC_matrix(i, 1) = NDC_centered[i].y;
        NDC_matrix(i, 2) = NDC_centered[i].z;
    }

    // Compute the covariance matrix
    Eigen::Matrix3d H = normed_matrix.transpose() * NDC_matrix;

    // Perform SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // Compute rotation matrix
    Eigen::Matrix3d R_eigen = V * U.transpose();

    // Ensure a right-handed coordinate system
    if (R_eigen.determinant() < 0) {
        V.col(2) *= -1;
        R_eigen = V * U.transpose();
    }

    // Compute translation vector
    Eigen::Vector3d centroid_normed_eigen(centroid_normed.x, centroid_normed.y, centroid_normed.z);
    Eigen::Vector3d centroid_NDC_eigen(centroid_NDC.x, centroid_NDC.y, centroid_NDC.z);
    Eigen::Vector3d t_eigen = centroid_NDC_eigen - R_eigen * centroid_normed_eigen;

    // Convert Eigen rotation matrix back to GLM
    glm::mat3 R;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = R_eigen(j, i); // Note the transpose due to storage differences

    glm::vec3 t(t_eigen(0), t_eigen(1), t_eigen(2));


    // Apply the transformation
    glm::vec3 transformed_points[3];

    for (int i = 0; i < 3; ++i) {
        transformed_points[i] = R * normedCameraColorCords[i] + t;
    }
 //   // 输出R矩阵
 //   std::cout << "R[0] : " << R[0][0] << " " << R[0][1] << " " << R[0][2] << std::endl;
 //   std::cout << "R[1] : " << R[1][0] << " " << R[1][1] << " " << R[1][2] << std::endl;
 //   std::cout << "R[2] : " << R[2][0] << " " << R[2][1] << " " << R[2][2] << std::endl;
 //   // 输出R的行列式
 //   std::cout << "R determinant: " << R_eigen.determinant() << std::endl;
 //   // 输出t向量
 //   std::cout << "t : " << t.x << " " << t.y << " " << t.z << std::endl;
	//// 输出normedCameraColorCords[0]
 //   std::cout << "normedCameraColorCords[0] : " << glm::vec3(normedCameraColorCords[0]).x << " " << glm::vec3(normedCameraColorCords[0]).y << " " << glm::vec3(normedCameraColorCords[0]).z << std::endl;
 //   // 输出R * normedCameraColorCords[0] + t
	//std::cout << "R * normedCameraColorCords[0] + t : " << (R * normedCameraColorCords[0] + t).x << " " << (R * normedCameraColorCords[0] + t).y << " " << (R * normedCameraColorCords[0] + t).z << std::endl;
     
    // Verify the result
    //for (int i = 0; i < 3; ++i) {
    //    std::cout << "Transformed Point " << i << ": " << transformed_points[i].x << ", "
    //        << transformed_points[i].y << ", " << transformed_points[i].z << std::endl;
    //    std::cout << "Target Point " << i << ": " << NDCfakemanCord[i].x << ", "
    //        << NDCfakemanCord[i].y << ", " << NDCfakemanCord[i].z << std::endl;
    //}

    // Initialize the transformation matrix to identity
    glm::mat4 camera2fakemanMat(1.0f); // Sets diagonal elements to 1, rest to 0
    R = glm::transpose(R);
    // Set rotation part (upper-left 3x3)
    // Remember: GLM matrices are column-major, so we set columns first
    camera2fakemanMat[0][0] = R[0][0]; // First column, first row
    camera2fakemanMat[0][1] = R[0][1]; // First column, second row
    camera2fakemanMat[0][2] = R[0][2]; // First column, third row
    camera2fakemanMat[0][3] = t.x;     // First column, fourth row (homogeneous coordinate)

    camera2fakemanMat[1][0] = R[1][0]; // Second column, first row
    camera2fakemanMat[1][1] = R[1][1]; // Second column, second row
    camera2fakemanMat[1][2] = R[1][2]; // Second column, third row
    camera2fakemanMat[1][3] = t.y;    // Second column, fourth row

    camera2fakemanMat[2][0] = R[2][0]; // Third column, first row
    camera2fakemanMat[2][1] = R[2][1]; // Third column, second row
    camera2fakemanMat[2][2] = R[2][2]; // Third column, third row
    camera2fakemanMat[2][3] = t.z;     // Third column, fourth row

    // Set translation part (last column)
    camera2fakemanMat[3][0] = 0.0f;    // Fourth column, first row
    camera2fakemanMat[3][1] = 0.0f;    // Fourth column, second row
    camera2fakemanMat[3][2] = 0.0f;    // Fourth column, third row
    camera2fakemanMat[3][3] = 1.0f;    // Fourth column, fourth row

    // 输出变换矩阵
	//std::cout << "camera2fakemanMat[0] : " << camera2fakemanMat[0].x << " " << camera2fakemanMat[0].y << " " << camera2fakemanMat[0].z << " " << camera2fakemanMat[0].w << std::endl;
	//std::cout << "camera2fakemanMat[1] : " << camera2fakemanMat[1].x << " " << camera2fakemanMat[1].y << " " << camera2fakemanMat[1].z << " " << camera2fakemanMat[1].w << std::endl;
	//std::cout << "camera2fakemanMat[2] : " << camera2fakemanMat[2].x << " " << camera2fakemanMat[2].y << " " << camera2fakemanMat[2].z << " " << camera2fakemanMat[2].w << std::endl;
	//std::cout << "camera2fakemanMat[3] : " << camera2fakemanMat[3].x << " " << camera2fakemanMat[3].y << " " << camera2fakemanMat[3].z << " " << camera2fakemanMat[3].w << std::endl;

	return camera2fakemanMat;
}
void Camera::setViewMatrix(glm::vec4 eye, glm::vec4 side, glm::vec4 up, glm::vec4 look_at)
{
    this->eye = eye;
    this->side = glm::normalize(side);
    this->up = glm::normalize(up);
    this->look_at = glm::normalize(look_at);

    /* Always Remember that for Right Handed Coordinate Systems, the Camera (initially aligned with World Reference frame)
     * has the look direction negative to that of the Z axis. Hence to get the basis vector in Z we have to invert the look vector.
     */
    view2world_mat = glm::mat4(side, up, -look_at, eye);
    //std::cout << "Eye: " << eye.x << " " << eye.y << " " << eye.z << std::endl;
    //std::cout << "Look At: " << look_at.x << " " << look_at.y << " " << look_at.z << std::endl;
    //std::cout << "Side: " << side.x << " " << side.y << " " << side.z << std::endl;
    //std::cout << "Up: " << up.x << " " << up.y << " " << up.z << std::endl;
}

void Camera::setUBO(std::vector<float>& cam_data)
{
    for(int i = 0; i < 21; i+=4)
    {
        if(i == 16)
        {
            cam_data.push_back(eye.x);
            cam_data.push_back(eye.y);
            cam_data.push_back(eye.z);
            cam_data.push_back(1);

            cam_data.push_back(this->view_plane_dist);
            return;
        }
        glm::vec4 temp = glm::column(view2world_mat, i/4);
        cam_data.push_back(temp.x);
        cam_data.push_back(temp.y);
        cam_data.push_back(temp.z);
        cam_data.push_back(temp.w);
    }
    is_changed = false;
}


void Camera::setOrientation(float zoom , float zenith, float azimuth)
{
    if(zenith == 0 && azimuth == 0)
    {
        if(zoom > 0)
            eye += look_at * mov_speed;
        else
            eye -= look_at * mov_speed;
        radius = glm::length(glm::vec3(eye.x, eye.y, eye.z));
        view2world_mat = glm::column(view2world_mat, 3, eye);
        is_changed = true;
        //std::cout << "Eye: " << eye.x << " " << eye.y << " " << eye.z << std::endl;
        //std::cout << "Look At: " << look_at.x << " " << look_at.y << " " << look_at.z << std::endl;
        //std::cout << "Side: " << side.x << " " << side.y << " " << side.z << std::endl;
        //std::cout << "Up: " << up.x << " " << up.y << " " << up.z << std::endl;
        return;
    }
    float new_zenith, new_azimuth;
    float pi2 = glm::pi<float>() * 2;

    new_zenith = glm::clamp(this->zenith + zenith * rotation_speed, 0.0f, glm::pi<float>());
    new_azimuth = this->azimuth + azimuth * rotation_speed;

    //Wrap Azimuth angle around 360 degrees.
    if(new_azimuth < 0)
        new_azimuth = pi2 - new_azimuth;
    else if(new_azimuth > pi2)
        new_azimuth = new_azimuth - pi2;

    //If no change in angle return
    if(new_zenith == this->zenith && new_azimuth == this->azimuth)
        return;

    this->zenith = new_zenith;
    this->azimuth = new_azimuth;

    /* Spherical to Cartesian. The Cartesian Coordinate Reference Frame used is the one used in Computer Graphics ( Z points forward, Y upwards)
     * rather than normal Mathematics where Z points upwards.
     *
     *  y = r * cos(zenith)
     *  z = r * sin(zenith) * cos(azimuth)
     *  x = r * sin(zenith) * sin(azimuth)
     */
    eye.x = this->radius * glm::sin(this->zenith) * glm::sin(this->azimuth);
    eye.y = this->radius * glm::cos(this->zenith);
    eye.z = this->radius * glm::sin(this->zenith) * glm::cos(this->azimuth);
    eye.w = 1;


    look_at = glm::vec4(-eye);
    look_at.w = 0;
    look_at = glm::normalize(look_at);

    /* When Zenith is 0 or PI, the look at vector collapses on to the world up.
     * We compute the side vector as (1,0,0) and rotate it around the world up based on
     * azimuth angle.
     */
    if(this->zenith == 0 || this->zenith == glm::pi<float>() )
    {
        side = glm::vec4(1,0,0,0);
        glm::mat4 rot_mat = glm::rotate(glm::mat4(1.0), this->azimuth, glm::vec3(0,1,0));
        side = rot_mat * side;
    }
    else
        side = glm::vec4(glm::cross(glm::vec3(look_at.x, look_at.y, look_at.z), glm::vec3(0,1,0)), 0);

    up = glm::vec4(glm::cross(glm::vec3(side.x, side.y, side.z), glm::vec3(look_at.x, look_at.y, look_at.z)), 0);
    side = glm::normalize(side);
    up = glm::normalize(up);

    view2world_mat = glm::mat4(side, up, -look_at, eye);
	//std::cout << "Eye: " << eye.x << " " << eye.y << " " << eye.z << std::endl;
	//std::cout << "Look At: " << look_at.x << " " << look_at.y << " " << look_at.z << std::endl;
	//std::cout << "Side: " << side.x << " " << side.y << " " << side.z << std::endl;
	//std::cout << "Up: " << up.x << " " << up.y << " " << up.z << std::endl;
    is_changed = true;
}
