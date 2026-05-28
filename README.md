# cv-boost-demo

智能车赛道边界检测与元素识别 - C++实现

## 功能概述

从 smtcgo2 (Python) 移植的C++版本，实现智能车视觉导航的核心算法。

## 模块说明

| 模块 | 说明 |
|------|------|
| `image_process` | 核心图像处理：八邻域边界跟踪、大津法阈值、形态学滤波 |
| `skeleton` | 骨架提取与分析：Zhang-Suen骨架化、分叉点/端点检测、圆环检测 |
| `find_line_lib/hello` | 测试模块 |
| `find_line_lib/get_start_point` | 赛道起点查找 |
| `find_line_lib/ring` | 圆环检测：发现圆环、准备入环、准备出环 |
| `find_line_lib/targetboard` | 目标板检测：红色掩码、角点检测、状态补线 |
| `find_line_lib/check_roadblock` | 路障检测：连通域分析、避障引导 |
| `find_line_lib/status_switcher` | 状态机管理 |
| `find_line_lib/check_road_element` | 赛道元素处理入口 |
| `find_line_lib/calculate_wheel_speeds` | 轮速计算：基于线斜率计算左右轮速 |

## 编译

```bash
pwsh -c './env.ps1 cmake --build --preset conan-release --parallel'
```

## 测试

```bash
./build/test_hello.exe
./build/test_get_start_point.exe
./build/test_status_switcher.exe
./build/test_calculate_wheel_speeds.exe
./build/test_skeleton.exe
```

## 依赖

- OpenCV 4.10.0
- Boost 1.87.0
- GTest 1.15.0

## 项目结构

```
include/
├── image_process.h          # 核心图像处理
├── skeleton.h               # 骨架提取与分析
├── utils.h                  # 工具函数
└── find_line_lib/           # find_line_lib库
    ├── common.h             # 公共类型：Point, StatusSwitcher, 枚举
    ├── hello.h
    ├── get_start_point.h
    ├── ring.h
    ├── targetboard.h
    ├── check_roadblock.h
    ├── check_road_element.h
    └── calculate_wheel_speeds.h

src/
├── main.cpp                 # 主程序：视频骨架处理状态机
├── utils.cpp
├── image_process.cpp
├── skeleton.cpp
└── find_line_lib/           # 实现文件
    ├── hello.cpp
    ├── get_start_point.cpp
    ├── ring.cpp
    ├── targetboard.cpp
    ├── check_roadblock.cpp
    ├── check_road_element.cpp
    └── calculate_wheel_speeds.cpp

tests/                       # Google Test测试
├── test_hello.cpp
├── test_get_start_point.cpp
├── test_status_switcher.cpp
├── test_calculate_wheel_speeds.cpp
└── test_skeleton.cpp

png/                         # 测试资源（从smtcgo2复制）
├── *.jpg                   # 测试图片
├── *.png
└── *.mp4                   # 测试视频
```

## 算法说明

### 八邻域边界跟踪

从图像底部中间开始，分别向左和向右搜索赛道边界点，使用8邻域搜索黑白跳变点，顺时针搜索左边线，逆时针搜索右边线。

### 骨架分析（视频轮廓骨架提取）

1. **预处理**：resize到160x120、灰度化、高斯滤波、二值化、膨胀腐蚀
2. **轮廓查找**：找最下方轮廓（面积≥20%），锁定真实道路区域
3. **骨架提取**：使用Zhang-Suen算法在道路区域内提取骨架
4. **特征点检测**：8邻域连接数≥3为分叉点，=1为端点
5. **圆环检测**：对骨架膨胀后用RETR_TREE检测轮廓层级，内孔=闭合环

### 圆环检测状态机

```
未发现 -> 已发现 -> 准备入环 -> 准备出环 -> 即将出环 -> 出环中 -> 未发现
```

### 路障检测

1. 反转图像并做连通域分析
2. 寻找靠近起点且最靠下的黑色障碍块
3. 根据障碍块相对位置生成避障引导线

### 轮速计算

基于最小二乘法拟合赛道中线，计算斜率方向，通过tanh函数平滑控制左右轮速差值。