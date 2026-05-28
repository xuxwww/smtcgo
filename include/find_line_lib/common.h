#pragma once

#include <cstdint>

constexpr int IMAGE_H_MAX = 480;
constexpr int IMAGE_W_MAX = 640;

constexpr int IMAGE_H = 120;
constexpr int IMAGE_W = 160;

namespace find_line_lib {

// ==================== 基础数据结构 ====================

// 二维坐标点
struct Point {
    int x;  // 列坐标
    int y;  // 行坐标

    Point() : x(0), y(0) {}
    Point(int _x, int _y) : x(_x), y(_y) {}
};

// 四个角点，用于描述四边形的四个顶点
struct CornerPoints {
    Point tl;  // 左上角 (top-left)
    Point tr;  // 右上角 (top-right)
    Point br;  // 右下角 (bottom-right)
    Point bl;  // 左下角 (bottom-left)
};

// ==================== 圆环相关枚举 ====================

// 圆环类型：表示圆环在左边还是右边
enum class RingType {
    None = 0,   // 不是圆环
    Left = 1,   // 圆环在左侧（左环）
    Right = 2   // 圆环在右侧（右环）
};

// 圆环状态机：描述车辆在圆环中的状态
enum class RingStatus {
    NotFound = 0,       // 还没发现圆环
    Discovered = 1,     // 刚发现圆环
    PrepareEnter = 2,   // 准备进入圆环
    PrepareExit = 3,    // 准备驶出圆环
    AboutToExit = 4,    // 即将驶出
    Exiting = 5         // 正在驶出
};

// ==================== 其他状态枚举 ====================

// 赛道类型识别：岔路口的类型
enum class ModelRecognitionStatus {
    NotFound = 0,  // 没识别到
    Left = 1,     // 左边岔路
    Straight = 2, // 直道
    Right = 3    // 右边岔路
};

// 目标板状态：描述目标板的检测状态
enum class TargetBoardStatus {
    NotFound = 0,    // 没找到目标板
    Discovered = 1,  // 刚发现目标板
    Passing = 2      // 正在通过目标板
};

// ==================== 状态切换器 ====================

// 状态管理器：跟踪所有关键状态
struct StatusSwitcher {
    RingStatus ring_status;              // 圆环状态
    RingType ring_type;                  // 圆环类型
    ModelRecognitionStatus model_status; // 赛道类型
    TargetBoardStatus target_board_status; // 目标板状态

    StatusSwitcher()
        : ring_status(RingStatus::NotFound),
          ring_type(RingType::None),
          model_status(ModelRecognitionStatus::NotFound),
          target_board_status(TargetBoardStatus::NotFound) {}
};

}