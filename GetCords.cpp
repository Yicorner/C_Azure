#include "GetCords.hpp"
#include "work.hpp"
#include <iostream>
#include <fstream>
#include <cassert> 


int Get3DCords::get_cords_with_filename(int x, int y, const std::string&  depth_filename, std::vector<std::vector<float>>& body3Dlocation_list) {
    /*
     * from pixel cordinates in color_image to 3D cordinates in color_image space cordinates by depth file
     */
    try
    {
		// Obtain calibration data only once

        // Create a transformation object

        int depth_width = 640;   // Set to your depth image width
        int depth_height = 576;  // Set to your depth image height

        //Read the depth image from the raw file
        size_t depth_size = depth_width * depth_height * sizeof(uint16_t);

        // Allocate a buffer to hold the depth data
        std::vector<uint8_t> depth_data(depth_size);
        std::string filename = depth_filename;
        // Open the depth image raw file
        std::ifstream depth_file(filename, std::ios::in | std::ios::binary);
        if (!depth_file)
        {
            std::cerr << "Failed to open " + filename << std::endl;
            (*device).close();
            assert(0 && "depth_image failed to open!");
        }

        // Read the depth data into the buffer
        depth_file.read(reinterpret_cast<char*>(depth_data.data()), depth_size);
        depth_file.close();

        // Verify that the correct amount of data was read
        if (depth_file.gcount() != static_cast<std::streamsize>(depth_size))
        {
            std::cerr << "Incomplete depth data read from file." << std::endl;
            (*device).close();
            assert(0 && "Incomplete depth data read from file.");
        }

        // Create a k4a::image object for the depth data
        k4a::image depth_image = k4a::image::create(
            K4A_IMAGE_FORMAT_DEPTH16,  // Depth image format
            depth_width,
            depth_height,
            depth_width * sizeof(uint16_t));  // Stride bytes

        // Copy the data into the k4a::image buffer
        uint8_t* depth_image_buffer = depth_image.get_buffer();
        memcpy(depth_image_buffer, depth_data.data(), depth_size);


        // Choose a 2D point in the color image (e.g., the center point)
        k4a_float2_t source_point2d;
        source_point2d.xy.x = static_cast<float>(x);
        source_point2d.xy.y = static_cast<float>(y);

        // Prepare the target point for the depth image
        k4a_float2_t target_point2d;

        // Use the calibration function to convert the color 2D point to depth 2D point
        bool valid = calibration.convert_color_2d_to_depth_2d(source_point2d, depth_image, &target_point2d);

        if (valid)
        {
            // Optionally, retrieve the depth value at the target point
            depth_width = depth_image.get_width_pixels();
            depth_height = depth_image.get_height_pixels();

            // Round the coordinates to nearest integer values
            int x = static_cast<int>(std::round(target_point2d.xy.x));
            int y = static_cast<int>(std::round(target_point2d.xy.y));

            if (x >= 0 && x < depth_width && y >= 0 && y < depth_height)
            {
                // Get the depth value at (x, y)
                uint16_t* depth_buffer = reinterpret_cast<uint16_t*>(depth_image.get_buffer());
                uint16_t depth_value = depth_buffer[y * depth_width + x];
            }
            else
            {
                std::cout << "Converted point is outside the depth image bounds." << std::endl;
                return 0;
            }
        }
        else
        {
            std::cout << "Conversion failed. The point may not have corresponding depth data." << std::endl;
            return 0;
        }
        /*let's get a new start!*/
        source_point2d = target_point2d;

        // Get the depth value at the source point
        uint16_t* depth_buffer = reinterpret_cast<uint16_t*>(depth_image.get_buffer());
        depth_width = depth_image.get_width_pixels();
        depth_height = depth_image.get_height_pixels();

        // Round the coordinates to nearest integer values
        int x = static_cast<int>(std::round(source_point2d.xy.x));
        int y = static_cast<int>(std::round(source_point2d.xy.y));

        if (x >= 0 && x < depth_width && y >= 0 && y < depth_height)
        {
            uint16_t depth_value = depth_buffer[y * depth_width + x];

            if (depth_value == 0)
            {
                std::cerr << "Invalid depth value at the source point." << std::endl;
                (*device).close();
                assert(0 && "Invalid depth value at the source point.");
            }

            float source_depth = static_cast<float>(depth_value);

            // Prepare the target point for the 3D coordinate
            k4a_float3_t target_point3d;

            // Use the calibration function to convert the 2D depth point to 3D in the color camera coordinate system
            bool valid = calibration.convert_2d_to_3d(
                source_point2d,
                source_depth,
                K4A_CALIBRATION_TYPE_DEPTH, // Source camera
                K4A_CALIBRATION_TYPE_COLOR, // Target camera
                &target_point3d);

            if (valid)
            {
                std::vector<float> result;
                std::cout << "Converted 3D point in color camera coordinate system: ("
                    << target_point3d.xyz.x << ", "
                    << target_point3d.xyz.y << ", "
                    << target_point3d.xyz.z << ")" << std::endl;
				result.push_back(target_point3d.xyz.x);
				result.push_back(target_point3d.xyz.y);
				result.push_back(target_point3d.xyz.z);
				body3Dlocation_list.push_back(result);
                return 1;
            }
            else
            {
                std::cerr << "Conversion failed. The point may not be valid in the target coordinate system." << std::endl;
                return 0;
            }
        }
        else
        {
            std::cerr << "Source point is outside the depth image bounds." << std::endl;
            return 0;
        }
    }
    catch (const k4a::error& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 0;
    }
    return 0;
}


int Get3DCords::get_cords_with_depth_image(int ox, int oy, k4a::image depth_image, std::vector<std::vector<float>>& body3Dlocation_list) {
    /*
    * from pixel cordinates in color_image to 3D cordinates in color_image space cordinates by k4a depth image
    */


	timer.start("Code segment get_cords_with_depth_image");
    // Create a transformation object

    int depth_width = depth_image.get_width_pixels();   // Set to your depth image width
    int depth_height = depth_image.get_height_pixels();  // Set to your depth image height

    //Read the depth image from the raw file
    size_t depth_size = depth_width * depth_height * sizeof(uint16_t);

    // Choose a 2D point in the color image (e.g., the center point)
    k4a_float2_t source_point2d;
    source_point2d.xy.x = static_cast<float>(ox);
    source_point2d.xy.y = static_cast<float>(oy);
    //std::cout << "Source point in color image: (" << source_point2d.xy.x << ", " << source_point2d.xy.y << ")" << std::endl;

    // Prepare the target point for the depth image
    k4a_float2_t target_point2d;

    // Use the calibration function to convert the color 2D point to depth 2D point
    bool valid = calibration.convert_color_2d_to_depth_2d(source_point2d, depth_image, &target_point2d);

    if (valid)
    {
        // Round the coordinates to nearest integer values
        int x = static_cast<int>(std::round(target_point2d.xy.x));
        int y = static_cast<int>(std::round(target_point2d.xy.y));

        if (x >= 0 && x < depth_width && y >= 0 && y < depth_height)
        {
            // Get the depth value at (x, y)
            uint16_t* depth_buffer = reinterpret_cast<uint16_t*>(depth_image.get_buffer());
            uint16_t depth_value = depth_buffer[y * depth_width + x];

            //std::cout << "Depth value at the converted point: " << depth_value << " mm" << std::endl;
        }
        else
        {
            std::cerr << "Converted point is outside the depth image bounds." << std::endl;
            timer.stop("Code segment get_cords_with_depth_image");

            return 0;
        }
    }
    else
    {
        std::cerr << "Conversion failed. The point may not have corresponding depth data. x: " << ox << ", y: " << oy << "; frame_id: " << Work::frame_id << std::endl;
        timer.stop("Code segment get_cords_with_depth_image");
        return 0;
    }

    /*let's get a new start!*/


    source_point2d = target_point2d;

    // Get the depth value at the source point
    uint16_t* depth_buffer = reinterpret_cast<uint16_t*>(depth_image.get_buffer());

    // Round the coordinates to nearest integer values
    int x = static_cast<int>(std::round(source_point2d.xy.x));
    int y = static_cast<int>(std::round(source_point2d.xy.y));

    if (x >= 0 && x < depth_width && y >= 0 && y < depth_height)
    {
        uint16_t depth_value = depth_buffer[y * depth_width + x];

        if (depth_value == 0)
        {
            std::cerr << "Invalid depth value at the source point." << std::endl;
            (*device).close();
            assert(0);
        }

        float source_depth = static_cast<float>(depth_value);

        // Prepare the target point for the 3D coordinate
        k4a_float3_t target_point3d;

        // Use the calibration function to convert the 2D depth point to 3D in the color camera coordinate system
        bool valid = calibration.convert_2d_to_3d(
            source_point2d,
            source_depth,
            K4A_CALIBRATION_TYPE_DEPTH, // Source camera
            K4A_CALIBRATION_TYPE_COLOR, // Target camera
            &target_point3d);

        if (valid)
        {
			std::vector<float> result;
            std::cout <<"x: " << ox << ", y : " << oy << ",Converted 3D point in color camera coordinate system : ("
                << target_point3d.xyz.x << ", "
                << target_point3d.xyz.y << ", "
                << target_point3d.xyz.z << ")" << std::endl;
            result.push_back(target_point3d.xyz.x);
            result.push_back(target_point3d.xyz.y);
            result.push_back(target_point3d.xyz.z);
			body3Dlocation_list.push_back(result);
            timer.stop("Code segment get_cords_with_depth_image");
            return 1;
        }
        else
        {
            std::cout << "Conversion failed. The point may not be valid in the target coordinate system." << std::endl;
            timer.stop("Code segment get_cords_with_depth_image");
            return 0;
        }
    }
    else
    {
        std::cout << "Source point is outside the depth image bounds." << std::endl;
        timer.stop("Code segment get_cords_with_depth_image");
        return 0;
    }
}

void Get3DCords::call_intrinsics() {
    // 获取彩色相机的内参
    k4a_calibration_camera_t color_camera_calibration = calibration.color_camera_calibration;

    // 获取彩色相机的内参参数
    k4a_calibration_intrinsics_t intrinsics = color_camera_calibration.intrinsics;
    k4a_calibration_intrinsic_parameters_t parameters = intrinsics.parameters;

    // 打印内参数据
    std::cout << "Color Camera Intrinsics:" << std::endl;
    std::cout << "cx (Principal Point x): " << parameters.param.cx << std::endl;
    std::cout << "cy (Principal Point y): " << parameters.param.cy << std::endl;
    std::cout << "fx (Focal Length x): " << parameters.param.fx << std::endl;
    std::cout << "fy (Focal Length y): " << parameters.param.fy << std::endl;

    std::cout << "Radial Distortion Coefficients:" << std::endl;
    std::cout << "k1: " << parameters.param.k1 << std::endl;
    std::cout << "k2: " << parameters.param.k2 << std::endl;
    std::cout << "k3: " << parameters.param.k3 << std::endl;
    std::cout << "k4: " << parameters.param.k4 << std::endl;
    std::cout << "k5: " << parameters.param.k5 << std::endl;
    std::cout << "k6: " << parameters.param.k6 << std::endl;

    std::cout << "Tangential Distortion Coefficients:" << std::endl;
    std::cout << "p1: " << parameters.param.p1 << std::endl;
    std::cout << "p2: " << parameters.param.p2 << std::endl;

    std::cout << "Metric Radius: " << parameters.param.metric_radius << std::endl;
}

std::vector<float> Get3DCords::get_cords_with_depth_image2(int ox, int oy, k4a::image depth_image) {
    /*
* from pixel cordinates in color_image to 3D cordinates in color_image space cordinates by k4a depth image
*/


    timer.start("Code segment get_cords_with_depth_image");

    int depth_width = depth_image.get_width_pixels();   // Set to your depth image width
    int depth_height = depth_image.get_height_pixels();  // Set to your depth image height

    //Read the depth image from the raw file
    size_t depth_size = depth_width * depth_height * sizeof(uint16_t);

    // Choose a 2D point in the color image (e.g., the center point)
    k4a_float2_t source_point2d;
    source_point2d.xy.x = static_cast<float>(ox);
    source_point2d.xy.y = static_cast<float>(oy);
    //std::cout << "Source point in color image: (" << source_point2d.xy.x << ", " << source_point2d.xy.y << ")" << std::endl;

    // Prepare the target point for the depth image
    k4a_float2_t target_point2d;

    // Use the calibration function to convert the color 2D point to depth 2D point
    timer.start("Code Segment convert_color_2d_to_depth_2d");
    bool valid = calibration.convert_color_2d_to_depth_2d(source_point2d, depth_image, &target_point2d);
    timer.stop("Code Segment convert_color_2d_to_depth_2d");


    if (valid)
    {
        // Round the coordinates to nearest integer values
        int x = static_cast<int>(std::round(target_point2d.xy.x));
        int y = static_cast<int>(std::round(target_point2d.xy.y));

        if (x >= 0 && x < depth_width && y >= 0 && y < depth_height)
        {
            // Get the depth value at (x, y)
            uint16_t* depth_buffer = reinterpret_cast<uint16_t*>(depth_image.get_buffer());
            uint16_t depth_value = depth_buffer[y * depth_width + x];

            //std::cout << "Depth value at the converted point: " << depth_value << " mm" << std::endl;
        }
        else
        {
            std::cerr << "Converted point is outside the depth image bounds." << std::endl;
            timer.stop("Code segment get_cords_with_depth_image");
            // 返回-10000，-10000，-10000
            std::vector<float> result = { -10001.0f, -10001.0f, -10001.0f };
            return result;
        }
    }
    else
    {
        std::cerr << "Conversion failed. The point may not have corresponding depth data. x: " << ox << ", y: " << oy << "; frame_id: " << Work::frame_id << std::endl;
        timer.stop("Code segment get_cords_with_depth_image");
        std::vector<float> result = { -10001.0f, -10001.0f, -10001.0f };
        return result;
    }

    /*let's get a new start!*/


    source_point2d = target_point2d;

    // Get the depth value at the source point
    uint16_t* depth_buffer = reinterpret_cast<uint16_t*>(depth_image.get_buffer());

    // Round the coordinates to nearest integer values
    int x = static_cast<int>(std::round(source_point2d.xy.x));
    int y = static_cast<int>(std::round(source_point2d.xy.y));

    if (x >= 0 && x < depth_width && y >= 0 && y < depth_height)
    {
        uint16_t depth_value = depth_buffer[y * depth_width + x];

        if (depth_value == 0)
        {
            std::cerr << "Invalid depth value at the source point." << std::endl;
            (*device).close();
            assert(0);
        }

        float source_depth = static_cast<float>(depth_value);

        // Prepare the target point for the 3D coordinate
        k4a_float3_t target_point3d;

        // Use the calibration function to convert the 2D depth point to 3D in the color camera coordinate system
		timer.start("Code Segment convert_2d_to_3d");
        bool valid = calibration.convert_2d_to_3d(
            source_point2d,
            source_depth,
            K4A_CALIBRATION_TYPE_DEPTH, // Source camera
            K4A_CALIBRATION_TYPE_COLOR, // Target camera
            &target_point3d);
        timer.stop("Code Segment convert_2d_to_3d");
        if (valid)
        {
            std::vector<float> result;
            std::cout << "x: " << ox << ", y : " << oy << ",Converted 3D point in color camera coordinate system : ("
                << target_point3d.xyz.x << ", "
                << target_point3d.xyz.y << ", "
                << target_point3d.xyz.z << ")" << std::endl;
            result.push_back(target_point3d.xyz.x);
            result.push_back(target_point3d.xyz.y);
            result.push_back(target_point3d.xyz.z);
            timer.stop("Code segment get_cords_with_depth_image");
            return result;
        }
        else
        {
            std::cout << "Conversion failed. The point may not be valid in the target coordinate system." << std::endl;
            timer.stop("Code segment get_cords_with_depth_image");
            std::vector<float> result = { -10001.0f, -10001.0f, -10001.0f };
            return result;
        }
    }
    else
    {
        std::cout << "Source point is outside the depth image bounds." << std::endl;
        timer.stop("Code segment get_cords_with_depth_image");
        std::vector<float> result = { -10001.0f, -10001.0f, -10001.0f };
        return result;
    }
}