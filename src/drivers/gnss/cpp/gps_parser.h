#ifndef GPS_PARSER_H
#define GPS_PARSER_H

#include <cstdint>
#include <vector>
#include <string>
#include <queue>
#include <mutex>
#include <optional>
#include "util.h"

struct GNGGA_S {
    double lat_rad;
    double lat_ang;
    double lon_rad;
    double lon_ang;
    double msl; /* Mean Sea Level, Unit: m */
};

struct GNVTG_S {
    double kph; /* Unit: km/h */
    double kph_east;
    double kph_north;
    double kph_up;
    double cogt_ang; /* Course Over Ground True */
    double cogt_rad;
};

struct GPS_MSG_S {
    SENSOR_TIME_S time;
    GNGGA_S gngga;
    GNVTG_S gnvtg;
    size_t msg_id;
};

class GPSParser {
private:
    static const size_t GPS_PARSE_LEN = 200; // 静态常量可在头文件初始化

    size_t msg_id_;
    GPS_MSG_S curr_msg_;
    std::queue<GPS_MSG_S> msg_queue_;
    std::mutex msg_queue_mtx_;

    int serial_port_;
    std::string serial_device_;
    speed_t baud_rate_;

    std::vector<std::string> format_gps_msg(std::string &msg);
    void parse_GNGGA(std::string &msg);
    void parse_GNVTG(std::string &msg);
    void parse_msg(std::string &msg);
    void push_msg();

public:
    GPSParser(std::string serial_device = "/dev/ttyUSB1", speed_t baud_rate = B115200);
    ~GPSParser();

    // 禁止拷贝构造和赋值（避免串口资源重复释放）
    GPSParser(const GPSParser &) = delete;
    GPSParser &operator=(const GPSParser &) = delete;

    void start_reading();
    std::optional<GPS_MSG_S> pop_msg();
};

#endif
