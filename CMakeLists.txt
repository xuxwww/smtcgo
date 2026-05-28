cmake_minimum_required(VERSION 3.16)
project(cv_boost_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_BINARY_DIR}/generators")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(GTest REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(complete_ring_process app/complete_ring_process.cpp)
target_link_libraries(complete_ring_process PRIVATE
    find_line_lib
    ${OpenCV_LIBS}
    Boost::boost
)

add_library(find_line_lib
    src/find_line_lib/hello.cpp
    src/find_line_lib/get_start_point.cpp
    src/find_line_lib/ring.cpp
    src/find_line_lib/targetboard.cpp
    src/find_line_lib/check_roadblock.cpp
    src/find_line_lib/check_road_element.cpp
    src/find_line_lib/calculate_wheel_speeds.cpp
    src/find_line_lib/skeleton.cpp
)
target_link_libraries(find_line_lib PRIVATE
    ${OpenCV_LIBS}
)

install(TARGETS complete_ring_process RUNTIME DESTINATION bin)

add_executable(app_hello app/test_hello.cpp)
target_link_libraries(app_hello PRIVATE find_line_lib)

add_executable(app_get_start_point app/test_get_start_point.cpp)
target_link_libraries(app_get_start_point PRIVATE find_line_lib ${OpenCV_LIBS})

add_executable(app_ring app/test_ring.cpp)
target_link_libraries(app_ring PRIVATE find_line_lib ${OpenCV_LIBS})

add_executable(app_check_roadblock app/test_check_roadblock.cpp)
target_link_libraries(app_check_roadblock PRIVATE find_line_lib ${OpenCV_LIBS})

add_executable(app_check_targetboard app/test_check_targetboard.cpp)
target_link_libraries(app_check_targetboard PRIVATE find_line_lib ${OpenCV_LIBS})

add_executable(app_status_switcher app/test_status_switcher.cpp)
target_link_libraries(app_status_switcher PRIVATE find_line_lib ${OpenCV_LIBS})

add_executable(app_skeleton_compare app/skeleton_compare.cpp)
target_link_libraries(app_skeleton_compare PRIVATE find_line_lib ${OpenCV_LIBS})