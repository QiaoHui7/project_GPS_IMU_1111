#include "imu_parser.h"
#include <iostream>
#include <cstdint>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iomanip>
#include <thread>

// --------------- Private ---------------

std::tuple<double, double, double> IMUParser::get_acc(unsigned char *datahex)
{
    unsigned char axl = datahex[0];
    unsigned char axh = datahex[1];
    unsigned char ayl = datahex[2];
    unsigned char ayh = datahex[3];
    unsigned char azl = datahex[4];
    unsigned char azh = datahex[5];
    double k_acc = 16.0;
    double acc_x = (axh << 8 | axl) / 32768.0 * k_acc;
    double acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc;
    double acc_z = (azh << 8 | azl) / 32768.0 * k_acc;
    if (acc_x >= k_acc) {
        acc_x -= 2 * k_acc;
    }
    if (acc_y >= k_acc) {
        acc_y -= 2 * k_acc;
    }
    if (acc_z >= k_acc) {
        acc_z -= 2 * k_acc;
    }
    return {acc_x, acc_y, acc_z};
}

std::tuple<double, double, double> IMUParser::get_gyro(unsigned char *datahex)
{
    unsigned char wxl = datahex[0];
    unsigned char wxh = datahex[1];
    unsigned char wyl = datahex[2];
    unsigned char wyh = datahex[3];
    unsigned char wzl = datahex[4];
    unsigned char wzh = datahex[5];
    double k_gyro = 2000.0;
    double gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro;
    double gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro;
    double gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro;
    if (gyro_x >= k_gyro) {
        gyro_x -= 2 * k_gyro;
    }
    if (gyro_y >= k_gyro) {
        gyro_y -= 2 * k_gyro;
    }
    if (gyro_z >= k_gyro) {
        gyro_z -= 2 * k_gyro;
    }
    return {gyro_x, gyro_y, gyro_z};
}

std::tuple<double, double, double> IMUParser::get_angle(unsigned char *datahex)
{
    unsigned char rxl = datahex[0];
    unsigned char rxh = datahex[1];
    unsigned char ryl = datahex[2];
    unsigned char ryh = datahex[3];
    unsigned char rzl = datahex[4];
    unsigned char rzh = datahex[5];
    double k_angle = 180.0;
    double angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle;
    double angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle;
    double angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle;
    if (angle_x >= k_angle) {
        angle_x -= 2 * k_angle;
    }
    if (angle_y >= k_angle) {
        angle_y -= 2 * k_angle;
    }
    if (angle_z >= k_angle) {
        angle_z -= 2 * k_angle;
    }
    return {angle_x, angle_y, angle_z};
}

std::tuple<double, double, double> IMUParser::get_mag(unsigned char *datahex)
{
    unsigned char hxl = datahex[0];
    unsigned char hxh = datahex[1];
    unsigned char hyl = datahex[2];
    unsigned char hyh = datahex[3];
    unsigned char hzl = datahex[4];
    unsigned char hzh = datahex[5];
    double mag_x = static_cast<double>(static_cast<int16_t>((static_cast<uint16_t>(hxh) << 8) | hxl)) * 0.00667;
    double mag_y = static_cast<double>(static_cast<int16_t>((static_cast<uint16_t>(hyh) << 8) | hyl)) * 0.00667;
    double mag_z = static_cast<double>(static_cast<int16_t>((static_cast<uint16_t>(hzh) << 8) | hzl)) * 0.00667;
    return {mag_x, mag_y, mag_z};
}

void IMUParser::imu_parse(unsigned char* imu_data, ssize_t imu_data_len)
{
    for (ssize_t i = 0; i < imu_data_len; i++) {
        if (field_label_ == 0) {
            if ((imu_data[i] == 0x55) && (field_pos_ == 0)) {
                check_sum_ = imu_data[i];
                field_pos_ = 1;
            } else if ((imu_data[i] == 0x51) && (field_pos_ == 1)) {
                check_sum_ += imu_data[i];
                field_label_ = 1;
                field_pos_ = 2;
            } else if ((imu_data[i] == 0x52) && (field_pos_ == 1)) {
                check_sum_ += imu_data[i];
                field_label_ = 2;
                field_pos_ = 2;
            } else if ((imu_data[i] == 0x53) && (field_pos_ == 1)) {
                check_sum_ += imu_data[i];
                field_label_ = 3;
                field_pos_ = 2;
            } else if ((imu_data[i] == 0x54) && (field_pos_ == 1)) {
                check_sum_ += imu_data[i];
                field_label_ = 4;
                field_pos_ = 2;
            } else {
                check_sum_ = 0;
                field_pos_ = 0;
                field_label_ = 0;
            }
            // field_pos_++; 这种写法不具有抗干扰性
        } else if (field_label_ == 1) { // acc
            if (field_pos_ < IMU_FIELD_LEN - 1) {
                acc_data_[field_pos_ - 2] = imu_data[i];
                check_sum_ += imu_data[i];
                field_pos_++;
            } else {
                if (imu_data[i] == (check_sum_ & 0xff)) {
                    auto [acc_x, acc_y, acc_z] = get_acc(acc_data_);
                    // std::cout << "acc_x: " << acc_x << ", acc_y: " << acc_y << ", acc_z: " << acc_z << std::endl;
                    curr_frame_.acc.x = acc_x;
                    curr_frame_.acc.y = acc_y;
                    curr_frame_.acc.z = acc_z;
                } else {
                    std::cerr << "acc data check failed." << std::endl;
                }
                check_sum_ = 0;
                field_pos_ = 0;
                field_label_ = 0;
            }
        } else if (field_label_ == 2) { // gyro
            if (field_pos_ < IMU_FIELD_LEN - 1) {
                gyro_data_[field_pos_ - 2] = imu_data[i];
                check_sum_ += imu_data[i];
                field_pos_++;
            } else {
                if (imu_data[i] == (check_sum_ & 0xff)) {
                    auto [gyro_x, gyro_y, gyro_z] = get_gyro(gyro_data_);
                    // std::cout << "gyro_x: " << gyro_x << ", gyro_y: " << gyro_y << ", gyro_z: " << gyro_z << std::endl;
                    curr_frame_.gyro.x = gyro_x;
                    curr_frame_.gyro.y = gyro_y;
                    curr_frame_.gyro.z = gyro_z;
                } else {
                    std::cout << "gyro data check failed." << std::endl;
                    // std::cout << "--------------------------------------------------" << std::endl;
                    // for (int j = 0; j <= i; j++) {
                    //     std::cout << std::setw(2) << std::setfill('0') << std::uppercase 
                    //         << std::hex << static_cast<int>(imu_data[j]) << " ";
                    // }
                    // std::cout << std::endl;
                    // std::cout << "--------------------------------------------------" << std::endl;
                }
                check_sum_ = 0;
                field_pos_ = 0;
                field_label_ = 0;
            }
        } else if (field_label_ == 3) { // angle
            if (field_pos_ < IMU_FIELD_LEN - 1) {
                angle_data_[field_pos_ - 2] = imu_data[i];
                check_sum_ += imu_data[i];
                field_pos_++;
            } else {
                if (imu_data[i] == (check_sum_ & 0xff)) {
                    auto [angle_x, angle_y, angle_z] = get_angle(angle_data_);
                    // std::cout << "angle_x: " << angle_x << ", angle_y: " << angle_y << ", angle_z: " << angle_z << std::endl;
                    curr_frame_.angle.x = angle_x;
                    curr_frame_.angle.y = angle_y;
                    curr_frame_.angle.z = angle_z;
                } else {
                    std::cout << "angle data check failed." << std::endl;
                }
                check_sum_ = 0;
                field_pos_ = 0;
                field_label_ = 0;
            }
        } else if (field_label_ == 4) { // magnet
            if (field_pos_ < IMU_FIELD_LEN - 1) {
                mag_data_[field_pos_ - 2] = imu_data[i];
                check_sum_ += imu_data[i];
                field_pos_++;
            } else {
                if (imu_data[i] == (check_sum_ & 0xff)) {
                    auto [mag_x, mag_y, mag_z] = get_mag(mag_data_);
                    // std::cout << "mag_x: " << mag_x << ", mag_y: " << mag_y << ", mag_z: " << mag_z << std::endl;
                    curr_frame_.mag.x = mag_x;
                    curr_frame_.mag.y = mag_y;
                    curr_frame_.mag.z = mag_z;
                    push_frame();
                } else {
                    std::cout << "mag data check failed." << std::endl;
                }
                check_sum_ = 0;
                field_pos_ = 0;
                field_label_ = 0;
            }
        }
    }
}

void IMUParser::push_frame()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    curr_frame_.time.ts_us = static_cast<uint64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;

    auto now = std::chrono::system_clock::now();
    // 转换为time_t类型（秒级精度），用于获取日期时间分量
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    // 转换为本地时间
    std::tm* local_time = std::localtime(&now_time);
    // 提取小时、分钟、秒（0-23, 0-59, 0-59）
    curr_frame_.time.hour = local_time->tm_hour;
    curr_frame_.time.min = local_time->tm_min;
    curr_frame_.time.sec = local_time->tm_sec;
    // 计算毫秒（0-999）
    curr_frame_.time.ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch() % std::chrono::seconds(1)
    ).count();

    curr_frame_.frame_id = frame_id_++;

    std::lock_guard<std::mutex> lock(frame_queue_mtx_);
    frame_queue_.push(curr_frame_);

    if (false) {
        std::cout << "--------------------------------------------------" << std::endl;
        std::cout << std::dec << "frame_id: " << curr_frame_.frame_id << std::endl;
        std::cout << "timestamp_us: " << curr_frame_.time.ts_us << std::endl;
        std::cout << curr_frame_.time.hour << ":" << curr_frame_.time.min << ":" << curr_frame_.time.sec << ":" << curr_frame_.time.ms << std::endl;
        std::cout << "[acc_x,   acc_y,   acc_z  ]: " << curr_frame_.acc.x << ", " << curr_frame_.acc.y << ", " << curr_frame_.acc.z << std::endl;
        std::cout << "[gyro_x,  gyro_y,  gyro_z ]: " << curr_frame_.gyro.x << ", " << curr_frame_.gyro.y << ", " << curr_frame_.gyro.z << std::endl;
        std::cout << "[angle_x, angle_y, angle_z]: " << curr_frame_.angle.x << ", " << curr_frame_.angle.y << ", " << curr_frame_.angle.z << std::endl;
        std::cout << "[mag_x,   mag_y,   mag_z  ]: " << curr_frame_.mag.x << ", " << curr_frame_.mag.y << ", " << curr_frame_.mag.z << std::endl;
    }

    return;
}

// --------------- Public ---------------

IMUParser::IMUParser(std::string serial_device, speed_t baud_rate)
    : serial_device_(std::move(serial_device)), baud_rate_(baud_rate), frame_id_(0), serial_port_(-1),
      field_label_(0), field_pos_(0), check_sum_(0) {
    memset(acc_data_, 0, sizeof(acc_data_));
    memset(gyro_data_, 0, sizeof(gyro_data_));
    memset(angle_data_, 0, sizeof(angle_data_));
    memset(mag_data_, 0, sizeof(mag_data_));
}

IMUParser::~IMUParser() {
    if (serial_port_ != -1) {
        close(serial_port_);
    }
}

void IMUParser::start_reading()
{
    std::thread([this]() {
        serial_port_ = open_serial_port(serial_device_.c_str(), baud_rate_);
        if (serial_port_ == -1) {
            return;
        }

        std::cout << "Start reading IMU serial data (Press Ctrl+C to exit)..." << std::endl;
        unsigned char buffer[IMU_BUFFER_LEN] = { 0 };
        while (true) {
            ssize_t read_bytes = read(serial_port_, buffer, IMU_BUFFER_LEN);
            if (read_bytes < 0) {
                std::cerr << "Error occurred while reading data!" << std::endl;
                break;
            }

            if (false) {
                for (ssize_t i = 0; i < read_bytes; i++) {
                    if (static_cast<int>(buffer[i]) == 0x55) {
                        std::cout << std::endl;
                    }
                    std::cout << std::setw(2) << std::setfill('0') << std::uppercase 
                            << std::hex << static_cast<int>(buffer[i]) << " ";
                }
            }

            imu_parse(buffer, read_bytes);
        }
    }).detach();
    // lambda表达式+分离线程detach(), 无需手动控制线程生命周期
}

std::optional<IMU_FRAME_S> IMUParser::pop_frame()
{
    std::lock_guard<std::mutex> lock(frame_queue_mtx_);
    if (!frame_queue_.empty()) {
        IMU_FRAME_S imu_frame = frame_queue_.front();
        frame_queue_.pop();
        return imu_frame;
    } 
    return std::nullopt;
}
