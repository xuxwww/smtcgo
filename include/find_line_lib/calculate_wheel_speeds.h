#pragma once

#include "common.h"
#include <tuple>

namespace find_line_lib {

// ==================== 轮速计算 ====================
// 根据中线斜率计算机器人左右轮的转速
// 实现差速转向：右转时左轮快右轮慢，左转时相反
//
// 参数：
//   line: 中线上的点数组（每行一个点）
//   line_size: 中线点的数量
//   base_speed: 基础速度（默认50）
//   max_gain_ratio: 最大增益比例（默认0.8）
//        这个比例乘以base_speed得到最大速度差
//
// 返回：
//   (左轮速度, 右轮速度) 的元组
//
// 算法：
//   1. 用最小二乘法拟合 x = slope*y + b，得到斜率slope
//   2. 中线在正中间竖直时，slope为0
//   3. 斜率越大说明中线越往前越偏右（要右转），斜率越小说明越往前越偏左（要左转）
//   4. 用tanh函数把斜率非线性映射到速度增益gain
//   5. 左轮速度 = base_speed + gain
//   6. 右轮速度 = base_speed - gain
//
// 例如：
//   slope > 0：中线越往前越偏右，需要右转 -> 左轮加速，右轮减速
//   slope < 0：中线越往前越偏左，需要左转 -> 左轮减速，右轮加速
std::tuple<float, float> calculate_wheel_speeds(const Point* line, int line_size,
                                                 float base_speed = 50.0f,
                                                 float max_gain_ratio = 0.8f);

}
