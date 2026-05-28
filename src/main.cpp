#include <opencv2/opencv.hpp>
#include <boost/version.hpp>

#include <iostream>
#include <cstring>
#include <vector>
#include <tuple>
#include <cmath>

#include "skeleton.h"
#include "video_skeleton_process.h"
#include "find_line_lib/common.h"

using namespace cv_boost;
using namespace find_line_lib;

struct FrameCache {
    cv::Mat frame;
};

int main() {
    std::cout << "Boost version: " << BOOST_VERSION << std::endl;
    std::cout << "OpenCV loaded OK" << std::endl;

    cv::VideoCapture cap("png/left_ring.mp4");
    if (!cap.isOpened()) {
        std::cout << "无法打开视频" << std::endl;
        return 1;
    }

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

    cv::namedWindow("Skeleton", cv::WINDOW_NORMAL);
    cv::resizeWindow("Skeleton", 400, 300);
    cv::namedWindow("Result", cv::WINDOW_NORMAL);
    cv::resizeWindow("Result", 400, 300);

    for (int current_idx = 0; current_idx < frame_count; ++current_idx) {
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
        cv::Rect road_roi(0, 0, 160, 120);
        int max_bottom_y = -1;
        cv::Point2i bottom_center(80, 60);

        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area < 160 * 120 * 0.2) continue;

            cv::Rect bounding = cv::boundingRect(contours[i]);
            int bottom_y = bounding.y + bounding.height;

            if (bottom_y > max_bottom_y) {
                max_bottom_y = bottom_y;
                bottom_center = cv::Point2i(bounding.x + bounding.width / 2, bottom_y);
            }
        }

        if (max_bottom_y > 0) {
            int road_min_y = max_bottom_y - 80;
            road_min_y = std::max(0, road_min_y);

            std::vector<cv::Point> road_contour;
            for (size_t i = 0; i < contours.size(); ++i) {
                double area = cv::contourArea(contours[i]);
                if (area < 160 * 120 * 0.2) continue;

                cv::Rect bounding = cv::boundingRect(contours[i]);
                int bottom_y = bounding.y + bounding.height;
                int top_y = bounding.y;

                if (bottom_y > max_bottom_y - 5 && top_y < max_bottom_y) {
                    for (auto& p : contours[i]) {
                        if (p.y >= road_min_y && p.y <= max_bottom_y) {
                            road_contour.push_back(p);
                        }
                    }
                }
            }

            std::vector<cv::Point> hull;
            if (road_contour.size() > 3) {
                cv::convexHull(road_contour, hull);
                std::vector<std::vector<cv::Point>> hull_contours = {hull};
                cv::drawContours(road_mask, hull_contours, -1, cv::Scalar(255), -1);
            }

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

            if (!skel_result.endpoint_points.empty()) {
                const char* status_name = status_machine.ring_status == RingStatus::NotFound ? "未发现" :
                                         status_machine.ring_status == RingStatus::Discovered ? "已发现" :
                                         status_machine.ring_status == RingStatus::PrepareEnter ? "准备入环" :
                                         status_machine.ring_status == RingStatus::PrepareExit ? "准备出环" :
                                         status_machine.ring_status == RingStatus::AboutToExit ? "即将出环" : "出环中";

                cv::Scalar target_color = purple;
                if (status_machine.ring_status == RingStatus::NotFound ||
                    status_machine.ring_status == RingStatus::Discovered ||
                    status_machine.ring_status == RingStatus::Exiting) {
                    int min_y = 120;
                    cv::Point target_pt(80, 60);
                    for (auto& p : skel_result.endpoint_points) {
                        int y = std::get<1>(p);
                        if (y < min_y) {
                            min_y = y;
                            target_pt = cv::Point(std::get<0>(p), y);
                        }
                    }
                    cv::circle(result_img, target_pt, 5, target_color, -1);
                } else if (status_machine.ring_status == RingStatus::PrepareEnter ||
                           status_machine.ring_status == RingStatus::AboutToExit) {
                    cv::Point target_pt(80, 60);
                    if (status_machine.ring_type == RingType::Left) {
                        int min_x = 160;
                        for (auto& p : skel_result.endpoint_points) {
                            int x = std::get<0>(p);
                            if (x < min_x) {
                                min_x = x;
                                target_pt = cv::Point(x, std::get<1>(p));
                            }
                        }
                    } else {
                        int max_x = 0;
                        for (auto& p : skel_result.endpoint_points) {
                            int x = std::get<0>(p);
                            if (x > max_x) {
                                max_x = x;
                                target_pt = cv::Point(x, std::get<1>(p));
                            }
                        }
                    }
                    cv::circle(result_img, target_pt, 5, target_color, -1);
                } else if (status_machine.ring_status == RingStatus::PrepareExit) {
                    if (skel_result.endpoint_points.size() == 1) {
                        auto& p = skel_result.endpoint_points[0];
                        cv::circle(result_img, cv::Point(std::get<0>(p), std::get<1>(p)), 5, target_color, -1);
                    }
                }
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
                       status_machine.ring_status == RingStatus::NotFound ? "未发现" :
                       status_machine.ring_status == RingStatus::Discovered ? "已发现" :
                       status_machine.ring_status == RingStatus::PrepareEnter ? "准备入环" :
                       status_machine.ring_status == RingStatus::PrepareExit ? "准备出环" :
                       status_machine.ring_status == RingStatus::AboutToExit ? "即将出环" : "出环中",
                       status_machine.ring_type == RingType::None ? "无" :
                       status_machine.ring_type == RingType::Left ? "左" : "右",
                       branch_count, endpoint_count, has_ring ? 1 : 0, ring_result.ring_center_x);
            }

            cv::imshow("Skeleton", skeleton_mat);
            cv::imshow("Result", result_img);
        }

        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q') break;
    }

    cv::destroyAllWindows();

    printf("\n==================================================\n");
    printf("视频处理结束，总帧数: %d\n", frame_count);
    printf("最终状态: %s\n",
           status_machine.ring_status == RingStatus::NotFound ? "未发现" :
           status_machine.ring_status == RingStatus::Discovered ? "已发现" :
           status_machine.ring_status == RingStatus::PrepareEnter ? "准备入环" :
           status_machine.ring_status == RingStatus::PrepareExit ? "准备出环" :
           status_machine.ring_status == RingStatus::AboutToExit ? "即将出环" : "出环中");
    printf("最终圆坏类型: %s\n",
           status_machine.ring_type == RingType::None ? "无" :
           status_machine.ring_type == RingType::Left ? "左" : "右");
    printf("==================================================\n");

    return 0;
}