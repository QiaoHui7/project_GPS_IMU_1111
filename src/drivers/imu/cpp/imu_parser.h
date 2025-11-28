#ifndef IMU_PARSER_H
#define IMU_PARSER_H

#include <cstdint>
#include <vector>
#include <string>
#include <queue>
#include <mutex>
#include <optional>
#include "util.h"

/* Unit: g, available value: 9.8m/s2 */
struct IMU_ACC_S {
    double x;
    double y;
    double z;
};

/* Unit: degree/s */
struct IMU_GYRO_S {
    double x;
    double y;
    double z;
};

/* Unit: degree */
struct IMU_ANGLE_S {
    double x;
    double y;
    double z;
};

/* Unit: uT */
struct IMU_MAG_S {
    double x;
    double y;
    double z;
};

struct IMU_FRAME_S {
    IMU_ACC_S acc;
    IMU_GYRO_S gyro;
    IMU_ANGLE_S angle;
    IMU_MAG_S mag;
    SENSOR_TIME_S time;
    size_t frame_id;
};

class IMUParser {
private:
    size_t frame_id_;
    IMU_FRAME_S curr_frame_;
    std::queue<IMU_FRAME_S> frame_queue_;
    std::mutex frame_queue_mtx_;

    int serial_port_;
    std::string serial_device_;
    speed_t baud_rate_;

    static const int IMU_FRAME_LEN = 44;
    static const int IMU_BUFFER_LEN = IMU_FRAME_LEN;
    static const int IMU_FIELD_LEN = 11;
    int field_label_;
    int field_pos_;
    int check_sum_;
    unsigned char acc_data_[8];
    unsigned char gyro_data_[8];
    unsigned char angle_data_[8];
    unsigned char mag_data_[8];

    std::tuple<double, double, double> get_acc(unsigned char* datahex);
    std::tuple<double, double, double> get_gyro(unsigned char* datahex);
    std::tuple<double, double, double> get_angle(unsigned char* datahex);
    std::tuple<double, double, double> get_mag(unsigned char* datahex);
    void imu_parse(unsigned char* imu_data, ssize_t imu_data_len);
    void push_frame();

public:
    IMUParser(std::string serial_device = "/dev/ttyUSB1", speed_t baud_rate = B115200);
    ~IMUParser();

    void start_reading();
    std::optional<IMU_FRAME_S> pop_frame();
};

#endif
