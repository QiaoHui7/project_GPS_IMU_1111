cmake_minimum_required(VERSION 3.20)

project (EKF)

set(CMAKE_CXX_STANDARD 17)
# 2. 强制要求该标准（若编译器不支持，CMake 会报错，避免隐式降级）
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 3. 禁用编译器扩展（可选，推荐：确保代码兼容标准 C++，不依赖 GCC/Clang 特有语法）
set(CMAKE_CXX_EXTENSIONS OFF)

# 全局编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
# 全局链接选项
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

set(SOURCES
    ./src_cpp/ekf_v0/ekfNavINS.cpp
    ./src_cpp/imu_parse.cpp
    ./src_cpp/gps_parse.cpp
    ./src_cpp/main_ekf.cpp
)

add_executable(EKF ${SOURCES})

# find eigen3
find_package(Eigen3 REQUIRED NO_MODULE)
# works on Ubuntu, failed on MAC
include_directories(${EIGEN3_INCLUDE_DIR})
# for MAC
# include_directories ( /usr/local/include/eigen3 )

target_include_directories(EKF
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/src_cpp/ekf_v0/
        ${PROJECT_SOURCE_DIR}/src_cpp/
)
