#pragma once

#include "common.h"
#include <tuple>
#include <opencv2/core/mat.hpp>

namespace find_line_lib {

// ==================== 轮速计算 ====================
std::tuple<float, float> calculate_wheel_speeds(const Point* line, int line_size,
                                                 float base_speed = 50.0f,
                                                 float max_gain_ratio = 0.8f);

std::tuple<float, float> calculate_wheel_speeds(const cv::Mat& image, float base_speed = 50.0f, float max_gain_ratio = 0.8f);

}
