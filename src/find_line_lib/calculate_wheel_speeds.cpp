#include "find_line_lib/calculate_wheel_speeds.h"

#include <cmath>
#include <tuple>

namespace find_line_lib {

// ==================== 轮速计算实现 ====================
// 核心算法：基于中线斜率的差速控制
//
// 原理：
//   假设我们用 Point {x, y} 描述中线上的点
//   拟合 x = k*y + b，而不是 y = k*x + b
//   这样正中间的竖直中线斜率为0，不会出现无穷大
//   如果斜率 > 0，说明 x 随 y 增大而增大（中线越往前越偏右）
//   如果斜率 < 0，说明 x 随 y 增大而减小（中线越往前越偏左）
//
// 算法步骤：
//   1. 收集所有中线点 (x_i, y_i)
//   2. 用最小二乘法拟合：x = k*y + b
//      k = (n*sum_xy - sum_y*sum_x) / (n*sum_yy - sum_y*sum_y)
//   3. 把斜率 k 通过 lambda 映射到增益比例
//   4. 增益 > 0 说明要右转，增益 < 0 说明要左转
//   5. 左右轮速度差由增益和 max_gain（= base_speed * max_gain_ratio）决定
std::tuple<float, float> calculate_wheel_speeds(const Point* line, int line_size,
                                                 float base_speed,
                                                 float max_gain_ratio) {
    // 数据点太少，直接返回基础速度
    if (line_size < 2) {
        return std::make_tuple(base_speed, base_speed);
    }

    // 收集统计数据，用于最小二乘法
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_yy = 0.0f;
    float sum_xy = 0.0f;

    for (int i = 0; i < line_size; ++i) {
        sum_x += static_cast<float>(line[i].x);
        sum_y += static_cast<float>(line[i].y);
        sum_yy += static_cast<float>(line[i].y) * line[i].y;
        sum_xy += static_cast<float>(line[i].x) * line[i].y;
    }

    // 拟合 x = k*y + b，让竖直中线的斜率为0
    float denom = static_cast<float>(line_size) * sum_yy - sum_y * sum_y;

    float slope = 0.0f;
    if (std::abs(denom) > 1e-5f) {
        slope = (static_cast<float>(line_size) * sum_xy - sum_y * sum_x) / denom;
    }

    // 计算最大速度增益
    float max_gain = base_speed * max_gain_ratio;

    // 限制斜率范围，防止极端情况
    if (slope > 5.0f) slope = 5.0f;
    if (slope < -5.0f) slope = -5.0f;

    // 只在这里替换斜率到增益比例的函数，例如线性、平方、三次方或tanh
    auto gain_function = [](float k) {
        constexpr float sensitivity = 2.0f;
        return std::tanh(sensitivity * k);
    };
    float gain = max_gain * gain_function(slope);

    // 计算左右轮速度
    // 增益为正时，左轮加速、右轮减速 -> 车辆右转
    // 增益为负时，左轮减速、右轮加速 -> 车辆左转
    float left_speed = base_speed + gain;
    float right_speed = base_speed - gain;

    return std::make_tuple(left_speed, right_speed);
}

}
