#ifndef GET_CORDS_HPP
#define GET_CORDS_HPP

#include <k4a/k4a.hpp>
int get_cords_with_filename(k4a::device& device, k4a_device_configuration_t& config, int x, int y, const std::string& depth_filename, std::vector<std::vector<float>>& body3Dlocation_list);
int get_cords_with_depth_image(k4a::device& device, k4a_device_configuration_t& config, int x, int y, k4a::image depth_image, std::vector<std::vector<float>>& body3Dlocation_list);
#endif