#include <opencv2/opencv.hpp>
#include <boost/version.hpp>

#include <iostream>
#include <cstring>
#include <vector>
#include <tuple>
#include <cmath>
#include <filesystem>
#include <string>
#include <queue>
#include <limits>
#include <algorithm>

#include "skeleton.h"
#include "video_skeleton_process.h"
#include "find_line_lib/common.h"

using namespace cv_boost;
using namespace find_line_lib;

struct FrameCache {
    cv::Mat frame;
};

static const int CONFIRM_THRESHOLD = 20;

static const char* ring_status_name(RingStatus status) {
    return status == RingStatus::NotFound ? "未发现" :
           status == RingStatus::Discovered ? "已发现" :
           status == RingStatus::PrepareEnter ? "准备入环" :
           status == RingStatus::PrepareExit ? "准备出环" :
           status == RingStatus::AboutToExit ? "即将出环" : "出环中";
}

static const char* ring_type_name(RingType type) {
    return type == RingType::None ? "无" : type == RingType::Left ? "左" : "右";
}

static bool choose_legacy_target_endpoint(const SkeletonAnalysisResult& skel_result,
                                          RingStatus status,
                                          RingType type,
                                          int width,
                                          int height,
                                          cv::Point& target_pt) {
    if (skel_result.endpoint_points.empty()) {
        return false;
    }

    if (status == RingStatus::NotFound ||
        status == RingStatus::Discovered ||
        status == RingStatus::Exiting) {
        int min_y = height;
        for (auto& p : skel_result.endpoint_points) {
            int x = std::get<0>(p);
            int y = std::get<1>(p);
            if (y < min_y) {
                min_y = y;
                target_pt = cv::Point(x, y);
            }
        }
        return true;
    }

    if (status == RingStatus::PrepareEnter || status == RingStatus::AboutToExit) {
        if (type == RingType::Left) {
            int min_x = width;
            for (auto& p : skel_result.endpoint_points) {
                int x = std::get<0>(p);
                int y = std::get<1>(p);
                if (x < min_x) {
                    min_x = x;
                    target_pt = cv::Point(x, y);
                }
            }
        } else {
            int max_x = -1;
            for (auto& p : skel_result.endpoint_points) {
                int x = std::get<0>(p);
                int y = std::get<1>(p);
                if (x > max_x) {
                    max_x = x;
                    target_pt = cv::Point(x, y);
                }
            }
        }
        return true;
    }

    if (status == RingStatus::PrepareExit && skel_result.endpoint_points.size() == 1) {
        auto& p = skel_result.endpoint_points[0];
        target_pt = cv::Point(std::get<0>(p), std::get<1>(p));
        return true;
    }

    return false;
}

static bool nearest_skeleton_point(const uint8_t* skeleton,
                                   int width,
                                   int height,
                                   cv::Point seed,
                                   cv::Point& nearest) {
    int best_dist = std::numeric_limits<int>::max();
    bool found = false;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (skeleton[y * width + x] == 0) {
                continue;
            }
            int dx = x - seed.x;
            int dy = y - seed.y;
            int dist = dx * dx + dy * dy;
            if (dist < best_dist) {
                best_dist = dist;
                nearest = cv::Point(x, y);
                found = true;
            }
        }
    }
    return found;
}

static bool find_skeleton_path(const uint8_t* skeleton,
                               int width,
                               int height,
                               cv::Point start,
                               cv::Point target,
                               std::vector<cv::Point>& path) {
    path.clear();
    if (start.x < 0 || start.x >= width || start.y < 0 || start.y >= height ||
        target.x < 0 || target.x >= width || target.y < 0 || target.y >= height) {
        return false;
    }
    if (skeleton[start.y * width + start.x] == 0 || skeleton[target.y * width + target.x] == 0) {
        return false;
    }

    const int total = width * height;
    std::vector<int> parent(total, -1);
    std::vector<uint8_t> visited(total, 0);
    std::queue<int> q;

    int start_idx = start.y * width + start.x;
    int target_idx = target.y * width + target.x;
    visited[start_idx] = 1;
    q.push(start_idx);

    const int dirs[8][2] = {
        {-1, -1}, {0, -1}, {1, -1},
        {-1, 0},           {1, 0},
        {-1, 1},  {0, 1},  {1, 1}
    };

    while (!q.empty()) {
        int idx = q.front();
        q.pop();
        if (idx == target_idx) {
            break;
        }

        int x = idx % width;
        int y = idx / width;
        for (auto& dir : dirs) {
            int nx = x + dir[0];
            int ny = y + dir[1];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                continue;
            }
            int nidx = ny * width + nx;
            if (visited[nidx] || skeleton[nidx] == 0) {
                continue;
            }
            visited[nidx] = 1;
            parent[nidx] = idx;
            q.push(nidx);
        }
    }

    if (!visited[target_idx]) {
        return false;
    }

    for (int idx = target_idx; idx != -1; idx = parent[idx]) {
        path.push_back(cv::Point(idx % width, idx / width));
        if (idx == start_idx) {
            break;
        }
    }
    std::reverse(path.begin(), path.end());
    return !path.empty() && path.front() == start && path.back() == target;
}

static double segment_length(cv::Point a, cv::Point b) {
    int dx = a.x - b.x;
    int dy = a.y - b.y;
    return std::sqrt(static_cast<double>(dx * dx + dy * dy));
}

static double polyline_length(const std::vector<cv::Point>& path) {
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        length += segment_length(path[i - 1], path[i]);
    }
    return length;
}

static cv::Point point_at_distance(const std::vector<cv::Point>& path, double distance) {
    if (path.empty()) {
        return cv::Point(0, 0);
    }
    if (path.size() == 1 || distance <= 0.0) {
        return path.front();
    }

    double walked = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        double seg_len = segment_length(path[i - 1], path[i]);
        if (walked + seg_len >= distance) {
            double t = seg_len > 0.0 ? (distance - walked) / seg_len : 0.0;
            int x = static_cast<int>(std::round(path[i - 1].x + (path[i].x - path[i - 1].x) * t));
            int y = static_cast<int>(std::round(path[i - 1].y + (path[i].y - path[i - 1].y) * t));
            return cv::Point(x, y);
        }
        walked += seg_len;
    }
    return path.back();
}

static bool select_folded_target_point(const uint8_t* skeleton,
                                       int width,
                                       int height,
                                       const SkeletonAnalysisResult& skel_result,
                                       RingStatus status,
                                       RingType type,
                                       cv::Point& legacy_target,
                                       cv::Point& folded_target) {
    if (!choose_legacy_target_endpoint(skel_result, status, type, width, height, legacy_target)) {
        return false;
    }

    folded_target = legacy_target;

    bool has_left_start = skel_result.left_start_x > 0 || skel_result.left_start_y > 0;
    bool has_right_start = skel_result.right_start_x > 0 || skel_result.right_start_y > 0;
    if (!has_left_start || !has_right_start) {
        return true;
    }

    cv::Point left_start(skel_result.left_start_x, skel_result.left_start_y);
    cv::Point right_start(skel_result.right_start_x, skel_result.right_start_y);
    cv::Point start_center((left_start.x + right_start.x) / 2,
                           (left_start.y + right_start.y) / 2);

    cv::Point left_skel, right_skel;
    std::vector<cv::Point> left_path, right_path;
    if (nearest_skeleton_point(skeleton, width, height, left_start, left_skel) &&
        nearest_skeleton_point(skeleton, width, height, right_start, right_skel) &&
        find_skeleton_path(skeleton, width, height, left_skel, legacy_target, left_path) &&
        find_skeleton_path(skeleton, width, height, right_skel, legacy_target, right_path)) {
        int li = static_cast<int>(left_path.size()) - 1;
        int ri = static_cast<int>(right_path.size()) - 1;
        while (li >= 0 && ri >= 0 && left_path[li] == right_path[ri]) {
            --li;
            --ri;
        }

        int merge_index = li + 1;
        int common_len = static_cast<int>(left_path.size()) - merge_index;
        if (merge_index >= 0 && merge_index < static_cast<int>(left_path.size()) && common_len >= 3) {
            std::vector<cv::Point> virtual_path;
            virtual_path.push_back(start_center);
            for (size_t i = static_cast<size_t>(merge_index); i < left_path.size(); ++i) {
                virtual_path.push_back(left_path[i]);
            }

            double total_length = polyline_length(virtual_path);
            folded_target = point_at_distance(virtual_path, total_length * 0.5);
            return true;
        }
    }

    cv::Point center_skel;
    std::vector<cv::Point> center_path;
    if (nearest_skeleton_point(skeleton, width, height, start_center, center_skel) &&
        find_skeleton_path(skeleton, width, height, center_skel, legacy_target, center_path)) {
        center_path.insert(center_path.begin(), start_center);
        double total_length = polyline_length(center_path);
        folded_target = point_at_distance(center_path, total_length * 0.5);
    }

    return true;
}

struct FrameData {
    uint8_t resized[IMAGE_H * IMAGE_W * 3];
    uint8_t gray[IMAGE_H * IMAGE_W];
    uint8_t blurred[IMAGE_H * IMAGE_W];
    uint8_t binary[IMAGE_H * IMAGE_W];
    uint8_t eroded[IMAGE_H * IMAGE_W];
    uint8_t dilated[IMAGE_H * IMAGE_W];
    uint8_t skeleton[IMAGE_H * IMAGE_W];
    uint8_t result[IMAGE_H * IMAGE_W * 3];
};

void create_debug_directories() {
    std::filesystem::create_directories("debug_frames/未发现");
    std::filesystem::create_directories("debug_frames/发现圆坏");
    std::filesystem::create_directories("debug_frames/已发现");
    std::filesystem::create_directories("debug_frames/准备入环");
    std::filesystem::create_directories("debug_frames/准备出环");
    std::filesystem::create_directories("debug_frames/即将出环");
    std::filesystem::create_directories("debug_frames/出环中");
}

void save_debug_images(const char* status, int frame_number,
                       const uint8_t* original,
                       const uint8_t* skeleton,
                       const uint8_t* result,
                       int width, int height) {
    char path[512];
    cv::Mat original_mat(height, width, CV_8UC3, const_cast<uint8_t*>(original));
    cv::Mat skeleton_mat(height, width, CV_8UC1, const_cast<uint8_t*>(skeleton));
    cv::Mat result_mat(height, width, CV_8UC3, const_cast<uint8_t*>(result));

    snprintf(path, sizeof(path), "debug_frames/%s/frame_%04d_原图.png", status, frame_number);
    cv::imwrite(path, original_mat);
    snprintf(path, sizeof(path), "debug_frames/%s/frame_%04d_骨架.png", status, frame_number);
    cv::imwrite(path, skeleton_mat);
    snprintf(path, sizeof(path), "debug_frames/%s/frame_%04d_结果.png", status, frame_number);
    cv::imwrite(path, result_mat);
}

void log_frame(const FrameInfo& info) {
    printf("[帧 %d] 状态=%s 圆坏类型=%s 分支=%d 端点=%d 有环=%d 环位置=%d\n",
           info.frame_number, info.status_name, info.ring_type_name,
           info.branch_count, info.endpoint_count,
           info.has_ring ? 1 : 0, info.ring_center_x);
}

static const uint8_t* find_bottom_contour(const uint8_t* contours_img, int width, int height,
                                          int& contour_area, int& bottom_y) {
    static uint8_t mask[IMAGE_H_MAX * IMAGE_W_MAX];
    memset(mask, 0, width * height);

    int min_area = width * height * 20 / 100;
    int max_bottom_y = -1;
    int best_area = 0;
    int best_x = 0, best_y = 0, best_w = 0, best_h = 0;

    int visited[IMAGE_H_MAX * IMAGE_W_MAX] = {0};

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (contours_img[y * width + x] == 255 && visited[y * width + x] == 0) {
                std::vector<std::tuple<int, int>> pixels;
                std::vector<std::tuple<int, int>> queue;
                queue.push_back(std::make_tuple(x, y));
                visited[y * width + x] = 1;

                while (!queue.empty()) {
                    auto [cx, cy] = queue.back();
                    queue.pop_back();
                    pixels.push_back(std::make_tuple(cx, cy));

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

                if (pixels.size() < 10) continue;

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

int process_video_skeleton(const char* video_path) {
    printf("视频骨架处理程序\n");
    printf("视频路径: %s\n", video_path);
    printf("注意: 此功能需要完整视频处理实现\n");
    return 0;
}

int main() {
    std::cout << "Boost version: " << BOOST_VERSION << std::endl;
    std::cout << "OpenCV loaded OK" << std::endl;

    cv::VideoCapture cap("png/left_ring.mp4");
    if (!cap.isOpened()) {
        std::cout << "无法打开视频" << std::endl;
        return 1;
    }

    ::create_debug_directories();

    int frame_count = 0;
    std::vector<cv::Mat> frame_cache;

    std::cout << "正在加载视频帧..." << std::endl;
    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        frame_cache.push_back(frame.clone());
        ++frame_count;
    }
    cap.release();
    std::cout << "已加载 " << frame_count << " 帧" << std::endl;

    StatusSwitcher status_machine;
    int ring_detect_count = 0;
    int ring_disappear_count = 0;
    int single_path_count = 0;
    int dual_path_count = 0;
    int confirm_threshold = 20;
    bool is_playing = true;
    int current_idx = 0;

    cv::namedWindow("Skeleton", cv::WINDOW_NORMAL);
    cv::resizeWindow("Skeleton", 400, 300);
    cv::namedWindow("Result", cv::WINDOW_NORMAL);
    cv::resizeWindow("Result", 400, 300);

    while (current_idx < frame_count) {
        cv::Mat& original_frame = frame_cache[current_idx];
        int frame_number = current_idx + 1;

        cv::Mat resized;
        cv::resize(original_frame, resized, cv::Size(160, 120));

        cv::Mat gray;
        cv::cvtColor(resized, gray, cv::COLOR_BGR2GRAY);

        cv::Mat blurred;
        cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);

        cv::Mat binary;
        cv::threshold(blurred, binary, 127, 255, cv::THRESH_BINARY);

        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat eroded;
        cv::erode(binary, eroded, element, cv::Point(-1, -1), 2);
        cv::Mat dilated;
        cv::dilate(eroded, dilated, element, cv::Point(-1, -1), 1);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dilated, contours, cv::noArray(), cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat result_img = resized.clone();
        cv::Mat road_mask = cv::Mat::zeros(120, 160, CV_8UC1);
        int max_bottom_y = -1;
        int best_contour_index = -1;
        int min_area = 160 * 120 * 20 / 100;
        int image_center_x = 80;

        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area < min_area) continue;

            cv::Rect bounding = cv::boundingRect(contours[i]);
            int bottom_y = bounding.y + bounding.height;
            int contour_center_x = bounding.x + bounding.width / 2;

            if (bottom_y > max_bottom_y) {
                max_bottom_y = bottom_y;
                best_contour_index = static_cast<int>(i);
            } else if (bottom_y == max_bottom_y && best_contour_index >= 0) {
                cv::Rect previous = cv::boundingRect(contours[best_contour_index]);
                int previous_center_x = previous.x + previous.width / 2;
                if (std::abs(contour_center_x - image_center_x) <
                    std::abs(previous_center_x - image_center_x)) {
                    best_contour_index = static_cast<int>(i);
                }
            }
        }

        if (best_contour_index >= 0) {
            cv::drawContours(result_img, contours, best_contour_index, cv::Scalar(0, 255, 0), 2);
            cv::drawContours(road_mask, contours, best_contour_index, cv::Scalar(255), -1);

            cv::Mat road_area;
            cv::bitwise_and(dilated, dilated, road_area, road_mask);

            uint8_t road_binary[120 * 160];
            for (int y = 0; y < 120; ++y) {
                for (int x = 0; x < 160; ++x) {
                    road_binary[y * 160 + x] = road_area.at<uint8_t>(y, x);
                }
            }

            uint8_t skeleton_result[120 * 160];
            skeletonize(road_binary, skeleton_result, 160, 120);

            cv::Mat skeleton_mat(120, 160, CV_8UC1, skeleton_result);

            uint8_t binary_for_start[120 * 160];
            for (int y = 0; y < 120; ++y) {
                for (int x = 0; x < 160; ++x) {
                    binary_for_start[y * 160 + x] = dilated.at<uint8_t>(y, x);
                }
            }

            auto skel_result = analyze_skeleton(skeleton_result, 160, 120, binary_for_start);

            cv::Mat skel_color;
            cv::cvtColor(skeleton_mat, skel_color, cv::COLOR_GRAY2BGR);

            cv::Scalar blue(255, 0, 0);
            cv::Scalar red(0, 0, 255);
            cv::Scalar green(0, 255, 0);
            cv::Scalar cyan(255, 255, 0);
            cv::Scalar purple(255, 0, 255);
            cv::Scalar yellow(0, 255, 255);

            for (auto& p : skel_result.endpoint_points) {
                int x = std::get<0>(p);
                int y = std::get<1>(p);
                cv::circle(result_img, cv::Point(x, y), 3, blue, -1);
            }

            for (auto& p : skel_result.bifurcation_points) {
                int x = std::get<0>(p);
                int y = std::get<1>(p);
                cv::circle(result_img, cv::Point(x, y), 5, red, -1);
            }

            if (skel_result.left_start_x > 0 || skel_result.left_start_y > 0) {
                cv::circle(result_img, cv::Point(skel_result.left_start_x, skel_result.left_start_y),
                          4, cyan, -1);
                cv::putText(result_img, "L",
                          cv::Point(skel_result.left_start_x - 5, skel_result.left_start_y - 8),
                          cv::FONT_HERSHEY_SIMPLEX, 0.3, cyan, 1);
            }
            if (skel_result.right_start_x > 0 || skel_result.right_start_y > 0) {
                cv::circle(result_img, cv::Point(skel_result.right_start_x, skel_result.right_start_y),
                          4, cyan, -1);
                cv::putText(result_img, "R",
                          cv::Point(skel_result.right_start_x - 5, skel_result.right_start_y - 8),
                          cv::FONT_HERSHEY_SIMPLEX, 0.3, cyan, 1);
            }

            auto ring_result = detect_ring(skeleton_result, 160, 120);

            if (ring_result.has_ring) {
                cv::circle(result_img, cv::Point(ring_result.ring_center_x, 60), 20, green, 2);
                std::string ring_text = "RING " + std::string(1, ring_result.ring_side);
                cv::putText(result_img, ring_text, cv::Point(ring_result.ring_center_x - 20, 50),
                          cv::FONT_HERSHEY_SIMPLEX, 0.4, green, 1);
            }

            cv::Point legacy_target;
            cv::Point folded_target;
            if (select_folded_target_point(skeleton_result, 160, 120, skel_result,
                                           status_machine.ring_status,
                                           status_machine.ring_type,
                                           legacy_target,
                                           folded_target)) {
                cv::circle(result_img, legacy_target, 5, purple, -1);
                cv::circle(result_img, folded_target, 5, yellow, -1);
                cv::putText(result_img, "F", cv::Point(folded_target.x + 4, folded_target.y - 4),
                          cv::FONT_HERSHEY_SIMPLEX, 0.3, yellow, 1);
            }

            bool has_ring = ring_result.has_ring;
            int branch_count = skel_result.branch_count;
            int endpoint_count = skel_result.endpoint_count;

            switch (status_machine.ring_status) {
                case RingStatus::NotFound:
                    if (has_ring) {
                        ring_detect_count++;
                        ring_disappear_count = 0;
                        if (ring_detect_count >= confirm_threshold) {
                            status_machine.ring_status = RingStatus::Discovered;
                            status_machine.ring_type = (ring_result.ring_side == 'l') ? RingType::Left : RingType::Right;
                            ring_detect_count = 0;
                            printf("[帧 %d] 未发现 -> 已发现 (%s圆环)\n",
                                   frame_number, status_machine.ring_type == RingType::Left ? "左" : "右");
                        }
                    } else {
                        ring_detect_count = 0;
                    }
                    break;

                case RingStatus::Discovered:
                    if (!has_ring) {
                        ring_disappear_count++;
                        ring_detect_count = 0;
                        if (ring_disappear_count >= confirm_threshold) {
                            status_machine.ring_status = RingStatus::PrepareEnter;
                            ring_disappear_count = 0;
                            printf("[帧 %d] 已发现 -> 准备入环\n", frame_number);
                        }
                    } else {
                        ring_disappear_count = 0;
                        status_machine.ring_type = (ring_result.ring_side == 'l') ? RingType::Left : RingType::Right;
                    }
                    break;

                case RingStatus::PrepareEnter:
                    if (branch_count == 1 && endpoint_count == 1) {
                        single_path_count++;
                        dual_path_count = 0;
                        if (single_path_count >= confirm_threshold) {
                            status_machine.ring_status = RingStatus::PrepareExit;
                            single_path_count = 0;
                            printf("[帧 %d] 准备入环 -> 准备出环\n", frame_number);
                        }
                    } else {
                        single_path_count = 0;
                    }
                    break;

                case RingStatus::PrepareExit:
                    if (endpoint_count >= 2) {
                        dual_path_count++;
                        single_path_count = 0;
                        if (dual_path_count >= confirm_threshold) {
                            status_machine.ring_status = RingStatus::AboutToExit;
                            dual_path_count = 0;
                            printf("[帧 %d] 准备出环 -> 即将出环\n", frame_number);
                        }
                    } else {
                        dual_path_count = 0;
                    }
                    break;

                case RingStatus::AboutToExit:
                    if (branch_count == 1 && endpoint_count == 1) {
                        single_path_count++;
                        dual_path_count = 0;
                        if (single_path_count >= confirm_threshold) {
                            status_machine.ring_status = RingStatus::Exiting;
                            status_machine.ring_type = RingType::None;
                            single_path_count = 0;
                            printf("[帧 %d] 即将出环 -> 出环中\n", frame_number);
                        }
                    } else {
                        single_path_count = 0;
                    }
                    break;

                case RingStatus::Exiting:
                    if (endpoint_count > 1) {
                        single_path_count++;
                        dual_path_count = 0;
                        if (single_path_count >= confirm_threshold) {
                            status_machine.ring_status = RingStatus::NotFound;
                            status_machine.ring_type = RingType::None;
                            single_path_count = 0;
                            printf("[帧 %d] 出环中 -> 未发现\n", frame_number);
                        }
                    } else {
                        single_path_count = 0;
                    }
                    break;
            }

            if (frame_number % 20 == 0) {
                printf("[帧 %d] 状态=%s 圆坏类型=%s 分支=%d 端点=%d 有环=%d 环位置=%d\n",
                       frame_number,
                       ring_status_name(status_machine.ring_status),
                       ring_type_name(status_machine.ring_type),
                       branch_count, endpoint_count, has_ring ? 1 : 0, ring_result.ring_center_x);
            }

            if (frame_number % 10 == 0) {
                ::save_debug_images(ring_status_name(status_machine.ring_status), frame_number,
                                    resized.data, skeleton_mat.data, result_img.data, 160, 120);
            }

            cv::Mat skeleton_display;
            cv::resize(skeleton_mat, skeleton_display, cv::Size(400, 300));
            cv::Mat result_display;
            cv::resize(result_img, result_display, cv::Size(400, 300));
            cv::imshow("Skeleton", skeleton_display);
            cv::imshow("Result", result_display);
        } else {
            bool has_ring = false;
            int branch_count = 0;
            int endpoint_count = 0;

            switch (status_machine.ring_status) {
                case RingStatus::NotFound:
                    ring_detect_count = 0;
                    break;
                case RingStatus::Discovered:
                    ring_disappear_count++;
                    ring_detect_count = 0;
                    if (ring_disappear_count >= confirm_threshold) {
                        status_machine.ring_status = RingStatus::PrepareEnter;
                        ring_disappear_count = 0;
                        printf("[帧 %d] 已发现 -> 准备入环\n", frame_number);
                    }
                    break;
                case RingStatus::PrepareEnter:
                case RingStatus::AboutToExit:
                    single_path_count = 0;
                    break;
                case RingStatus::PrepareExit:
                    dual_path_count = 0;
                    break;
                case RingStatus::Exiting:
                    single_path_count = 0;
                    break;
            }

            if (frame_number % 20 == 0) {
                printf("[帧 %d] 状态=%s 圆坏类型=%s 分支=%d 端点=%d 有环=%d 环位置=%d\n",
                       frame_number, ring_status_name(status_machine.ring_status),
                       ring_type_name(status_machine.ring_type), branch_count, endpoint_count,
                       has_ring ? 1 : 0, 0);
            }

            cv::Mat empty_skeleton = cv::Mat::zeros(120, 160, CV_8UC1);
            if (frame_number % 10 == 0) {
                ::save_debug_images(ring_status_name(status_machine.ring_status), frame_number,
                                    resized.data, empty_skeleton.data, result_img.data, 160, 120);
            }

            cv::Mat result_display;
            cv::resize(result_img, result_display, cv::Size(400, 300));
            cv::imshow("Result", result_display);
        }

        int wait_time = is_playing ? 1 : 0;
        char key = cv::waitKey(wait_time) & 0xFF;
        if (key == 'q') break;
        if (key == ' ') {
            is_playing = !is_playing;
            printf("%s (帧 %d)\n", is_playing ? "继续播放" : "已暂停", frame_number);
        } else if (key == 'd' || key == 83) {
            current_idx = std::min(current_idx + 100, frame_count - 1);
            is_playing = false;
        } else if (key == 'a' || key == 81) {
            current_idx = std::max(current_idx - 100, 0);
            is_playing = false;
        }

        if (is_playing) {
            ++current_idx;
        }
    }

    cv::destroyAllWindows();

    printf("\n==================================================\n");
    printf("视频处理结束，总帧数: %d\n", frame_count);
    printf("最终状态: %s\n",
           ring_status_name(status_machine.ring_status));
    printf("最终圆坏类型: %s\n", ring_type_name(status_machine.ring_type));
    printf("==================================================\n");

    return 0;
}
