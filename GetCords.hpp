#ifndef GET_CORDS_HPP
#define GET_CORDS_HPP

#include <k4a/k4a.hpp>
#include "MultiTimer.hpp"

class Get3DCords {
public:
	MultiTimer& timer = MultiTimer::getInstance();
	k4a::calibration calibration;
	k4a::device* device;
	k4a_device_configuration_t* config;
	Get3DCords(k4a::device& device, k4a_device_configuration_t& config) {
		this->device = &device;
		this->config = &config;
		calibration = device.get_calibration(config.depth_mode, config.color_resolution);
	};
	~Get3DCords() {};
	int get_cords_with_filename(int x, int y, const std::string& depth_filename, std::vector<std::vector<float>>& body3Dlocation_list);
	int get_cords_with_depth_image(int x, int y, k4a::image depth_image, std::vector<std::vector<float>>& body3Dlocation_list);
	void call_intrinsics();
};
#endif