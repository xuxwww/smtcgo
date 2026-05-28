#pragma once

#include "common.h"
#include <tuple>
#include <opencv2/core/mat.hpp>

namespace find_line_lib {

// ==================== 轮速计算 ====================
std::tuple<float, float> calculate_wheel_speeds(const Point* line, int line_size,
                                                 float base_speed = 50.0f,
                                                 float max_gain_ratio = 0.8f);

std::tuple<float, float> calculate_wheel_speeds(const cv::Mat& image);

// ==================== 帧处理 ====================
// 处理单帧：预处理 → 轮廓检测 → 骨架分析 → 环检测 → 状态机 → 绘制标注
// 内部用 static 变量维护状态机（ring_status, ring_type, 各计数器）
// 返回 (result_img, skeleton_img)
void process_frame(const cv::Mat& image);

}
