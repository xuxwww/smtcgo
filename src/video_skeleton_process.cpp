#include "video_skeleton_process.h"
#include "skeleton.h"
#include "find_line_lib/status_switcher.h"

#include <cstdio>
#include <cstring>
#include <vector>
#include <tuple>
#include <algorithm>

namespace cv_boost {

using namespace find_line_lib;

static const int CONFIRM_THRESHOLD = 20;

// ==================== 每帧数据缓存 ====================
// 用于暂存处理过程中的图像数据，避免重复分配
struct FrameData {
    uint8_t resized[IMAGE_H * IMAGE_W * 3];   // 缩放后的彩色图
    uint8_t gray[IMAGE_H * IMAGE_W];          // 灰度图
    uint8_t blurred[IMAGE_H * IMAGE_W];       // 模糊后的图
    uint8_t binary[IMAGE_H * IMAGE_W];        // 二值图
    uint8_t eroded[IMAGE_H * IMAGE_W];        // 腐蚀后的图
    uint8_t dilated[IMAGE_H * IMAGE_W];       // 膨胀后的图
    uint8_t skeleton[IMAGE_H * IMAGE_W];      // 骨架图
    uint8_t result[IMAGE_H * IMAGE_W * 3];    // 结果图（彩色）
};

// ==================== 创建调试目录 ====================
// 用于保存调试图片（当前为空实现）
void create_debug_directories() {
    // TODO: 创建debug用的输出目录
}

// ==================== 保存调试图片 ====================
// 把处理过程中的关键图像保存下来，方便分析
// 参数：
//   status: 当前状态名称
//   frame_number: 帧号
//   original: 原始帧
//   skeleton: 骨架图
//   result: 结果图
//   width, height: 图像尺寸
void save_debug_images(const char* status, int frame_number,
                       const uint8_t* original,
                       const uint8_t* skeleton,
                       const uint8_t* result,
                       int width, int height) {
    // TODO: 实现调试图片保存功能
}

// ==================== 打印帧信息 ====================
// 打印一帧的处理结果
void log_frame(const FrameInfo& info) {
    printf("[帧 %d] 状态=%s 圆坏类型=%s 分支=%d 端点=%d 有环=%d 环位置=%d\n",
           info.frame_number, info.status_name, info.ring_type_name,
           info.branch_count, info.endpoint_count,
           info.has_ring ? 1 : 0, info.ring_center_x);
}

// ==================== 找最下方的轮廓 ====================
// 从二值图像中找最下方的白色连通域
// 这用于找到赛道的主体部分（通常在图像下方）
//
// 原理：
//   1. 用BFS遍历所有白色连通域
//   2. 计算每个连通域的外接矩形和面积
//   3. 取外接矩形底部y值最大的那个（最下方的轮廓）
//   4. 如果底部y值相同，取面积更大的
//
// 参数：
//   contours_img: 二值图像（255=白）
//   width, height: 图像尺寸
//   contour_area: 输出，最下方轮廓的面积
//   bottom_y: 输出，最下方轮廓的底部y坐标
//
// 返回：包含最下方轮廓的掩码图像
static const uint8_t* find_bottom_contour(const uint8_t* contours_img, int width, int height,
                                          int& contour_area, int& bottom_y) {
    static uint8_t mask[IMAGE_H_MAX * IMAGE_W_MAX];
    memset(mask, 0, width * height);

    int min_area = width * height * 20 / 100;  // 最小面积阈值（图像的20%）
    int max_bottom_y = -1;
    int best_area = 0;
    int best_x = 0, best_y = 0, best_w = 0, best_h = 0;

    int visited[IMAGE_H_MAX * IMAGE_W_MAX] = {0};

    // 遍历所有白色像素
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (contours_img[y * width + x] == 255 && visited[y * width + x] == 0) {
                // BFS找连通域
                std::vector<std::tuple<int, int>> pixels;
                std::vector<std::tuple<int, int>> queue;
                queue.push_back(std::make_tuple(x, y));
                visited[y * width + x] = 1;

                while (!queue.empty()) {
                    auto [cx, cy] = queue.back();
                    queue.pop_back();
                    pixels.push_back(std::make_tuple(cx, cy));

                    // 8邻域扩展
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            if (dx == 0 && dy == 0) continue;
                            int nx = cx + dx;
                            int ny = cy + dy;
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                if (contours_img[ny * width + nx] == 255 && visited[ny * width + nx] == 0) {
                                    visited[ny * width + nx] = 1;
                                    queue.push_back(std::make_tuple(nx, ny));
                                }
                            }
                        }
                    }
                }

                // 像素太少不要（噪点）
                if (pixels.size() < 10) continue;

                // 计算外接矩形
                int min_x = width, max_x = 0, min_y = height, max_y = 0;
                for (auto& p : pixels) {
                    int px = std::get<0>(p);
                    int py = std::get<1>(p);
                    min_x = std::min(min_x, px);
                    max_x = std::max(max_x, px);
                    min_y = std::min(min_y, py);
                    max_y = std::max(max_y, py);
                }

                int area = static_cast<int>(pixels.size());
                int cur_bottom_y = min_y + (max_y - min_y);

                // 取最下方且面积最大的轮廓
                if (cur_bottom_y > max_bottom_y ||
                    (cur_bottom_y == max_bottom_y && area > best_area)) {
                    max_bottom_y = cur_bottom_y;
                    best_area = area;
                    best_x = min_x;
                    best_y = min_y;
                    best_w = max_x - min_x + 1;
                    best_h = max_y - min_y + 1;
                }
            }
        }
    }

    contour_area = best_area;
    bottom_y = max_bottom_y;

    // 生成掩码
    if (best_area > 0) {
        for (int y = best_y; y < best_y + best_h; ++y) {
            for (int x = best_x; x < best_x + best_w; ++x) {
                if (contours_img[y * width + x] == 255) {
                    mask[y * width + x] = 255;
                }
            }
        }
    }

    return mask;
}

// ==================== 视频骨架处理主函数 ====================
// 这是主要入口，用于处理整个视频
// 当前实现是一个占位符，需要完整实现
int process_video_skeleton(const char* video_path) {
    printf("视频骨架处理程序\n");
    printf("视频路径: %s\n", video_path);
    printf("注意: 此功能需要完整视频处理实现\n");
    return 0;
}

}