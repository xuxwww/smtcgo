#pragma once

#include "find_line_lib/common.h"
#include <tuple>

namespace cv_boost {

// ==================== 帧信息结构 ====================
// 用于描述每一帧的处理结果
struct FrameInfo {
    int frame_number;          // 帧号（从1开始）
    int total_frames;         // 总帧数
    const char* status_name;   // 当前状态名称（中文）
    const char* ring_type_name; // 圆环类型名称（中文）
    int branch_count;         // 分支数量
    int endpoint_count;      // 端点数量
    bool has_ring;            // 是否检测到圆环
    int ring_center_x;        // 圆环中心x坐标
};

// ==================== 调试相关函数 ====================

// 创建调试用的输出目录
void create_debug_directories();

// 保存调试图片（原始图、骨架图、结果图）
void save_debug_images(const char* status, int frame_number,
                       const uint8_t* original,
                       const uint8_t* skeleton,
                       const uint8_t* result,
                       int width, int height);

// 打印帧信息到控制台
void log_frame(const FrameInfo& info);

// ==================== 视频处理主函数 ====================
// 处理视频文件，提取骨架并分析
// 参数：视频文件路径
// 返回：0成功，-1失败
int process_video_skeleton(const char* video_path);

}