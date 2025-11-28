#include "gps_parser.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iomanip>
#include <regex>
#include <cmath>
#include <thread>

// --------------- Private ---------------

std::vector<std::string> GPSParser::format_gps_msg(std::string &msg)
{
    size_t comma_pos = msg.find(',');
    // remove prefix "$GNxxx,"
    if (comma_pos != std::string::npos) {
        msg.erase(0, comma_pos + 1);
    }
    // remove suffix \r\n
    if (msg.size() >= 2 && msg.substr(msg.size() - 2) == "\r\n") {
        msg.erase(msg.end() - 2, msg.end());
    }

    std::vector<std::string> result;
    std::string current;
    for (size_t i = 0; i < msg.size(); ++i) {
        char c = msg[i];
        if (c == ',') {
            if (current.empty()) {
                current = "000000.000";
            }
            result.push_back(current); // 即使 current 为空也要 push_back
            current.clear();
        } else {
            current += c;
        }
    }
    result.push_back(current); // 最后一个字段
    return result;
} 

void GPSParser::parse_GNGGA(std::string &msg)
{
    std::vector<std::string> GNGGA_fields = format_gps_msg(msg);

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    curr_msg_.time.ts_us = static_cast<uint64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
    if (true) {
        auto now = std::chrono::system_clock::now();
        // 转换为time_t类型（秒级精度），用于获取日期时间分量
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        // 转换为本地时间
        std::tm* local_time = std::localtime(&now_time);
        // 提取小时、分钟、秒（0-23, 0-59, 0-59）
        curr_msg_.time.hour = local_time->tm_hour;
        curr_msg_.time.min = local_time->tm_min;
        curr_msg_.time.sec = local_time->tm_sec;
        // 计算毫秒（0-999）
        curr_msg_.time.ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch() % std::chrono::seconds(1)
        ).count();
    } else {
        curr_msg_.time.hour = std::stoi(GNGGA_fields[0].substr(0, 2));
        curr_msg_.time.min = std::stoi(GNGGA_fields[0].substr(2, 2));
        curr_msg_.time.sec = std::stoi(GNGGA_fields[0].substr(4, 2));
        curr_msg_.time.ms = std::stoi(GNGGA_fields[0].substr(7, 3));
    }
    // if (hour < 0 || hour >= 24 || minute < 0 || minute >= 60 || 
    //     second < 0 || second >= 60 || ms < 0 || ms >= 1000) {
    //     std::cerr << "UTC value invalid." std::endl;
    // }

    curr_msg_.gngga.lat_ang = std::stoi(GNGGA_fields[1].substr(0, 2)) + std::stod(GNGGA_fields[1].substr(2)) / 60.0;
    curr_msg_.gngga.lat_rad = curr_msg_.gngga.lat_ang * M_PI / 180.0;
    if (GNGGA_fields[2] != "N") {
        curr_msg_.gngga.lat_ang = -curr_msg_.gngga.lat_ang;
        curr_msg_.gngga.lat_rad = -curr_msg_.gngga.lat_rad;
    }

    curr_msg_.gngga.lon_ang = std::stoi(GNGGA_fields[3].substr(0, 3)) + std::stod(GNGGA_fields[3].substr(3)) / 60.0;
    curr_msg_.gngga.lon_rad = curr_msg_.gngga.lon_ang * M_PI / 180.0;
    if (GNGGA_fields[4] != "E") {
        curr_msg_.gngga.lon_ang = -curr_msg_.gngga.lon_ang;
        curr_msg_.gngga.lon_rad = -curr_msg_.gngga.lon_rad;
    }

    curr_msg_.gngga.msl = std::stod(GNGGA_fields[8]);

    return;
}

void GPSParser::parse_GNVTG(std::string &msg)
{
    std::vector<std::string> GNVTG_fields = format_gps_msg(msg);

    curr_msg_.gnvtg.kph = std::stod(GNVTG_fields[6]);
    curr_msg_.gnvtg.cogt_ang = std::stod(GNVTG_fields[0]);
    curr_msg_.gnvtg.cogt_rad = curr_msg_.gnvtg.cogt_ang * M_PI / 180.0;
    curr_msg_.gnvtg.kph_north = curr_msg_.gnvtg.kph * std::cos(curr_msg_.gnvtg.cogt_rad);
    curr_msg_.gnvtg.kph_east = curr_msg_.gnvtg.kph * std::sin(curr_msg_.gnvtg.cogt_rad);
    curr_msg_.gnvtg.kph_up = 0;

    push_msg();
    if (false) {
        std::cout << "--------------------------------------------------" << std::endl;
        std::cout << "msg_id: " << curr_msg_.msg_id << std::endl;
        std::cout << "timestamp_us: " << curr_msg_.time.ts_us << std::endl;
        std::cout << "time: " << curr_msg_.time.hour << ":" << curr_msg_.time.min << ":" << curr_msg_.time.sec << "." << curr_msg_.time.ms << std::endl;
        std::cout << "lat_ang: " << curr_msg_.gngga.lat_ang << std::endl;
        std::cout << "lat_rad: " << curr_msg_.gngga.lat_rad << std::endl;
        std::cout << "lon_ang: " << curr_msg_.gngga.lon_ang << std::endl;
        std::cout << "lon_rad: " << curr_msg_.gngga.lon_rad << std::endl;
        std::cout << "msl: " << curr_msg_.gngga.msl << std::endl;
        std::cout << "kph: " << curr_msg_.gnvtg.kph << std::endl;
        std::cout << "kph_east: " << curr_msg_.gnvtg.kph_east << std::endl;
        std::cout << "kph_north: " << curr_msg_.gnvtg.kph_north << std::endl;
        std::cout << "kph_up: " << curr_msg_.gnvtg.kph_up << std::endl;
        std::cout << "cogt_ang: " << curr_msg_.gnvtg.cogt_ang << std::endl;
        std::cout << "cogt_rad: " << curr_msg_.gnvtg.cogt_rad << std::endl;
    }

    return;
}

void GPSParser::parse_msg(std::string &msg)
{
    if (msg.find("$GNGGA") == 0) {
        parse_GNGGA(msg);
    } else if (msg.find("$GNVTG") == 0) {
        parse_GNVTG(msg);
    }
    return;
}

void GPSParser::push_msg()
{
    std::lock_guard<std::mutex> lock(msg_queue_mtx_);
    curr_msg_.msg_id = msg_id_++;
    msg_queue_.push(curr_msg_);

    return;
}

// --------------- Public ---------------

GPSParser::GPSParser(std::string serial_device, speed_t baud_rate)
    : serial_device_(std::move(serial_device)), baud_rate_(baud_rate), msg_id_(0), serial_port_(-1) {}

GPSParser::~GPSParser()
{
    if (serial_port_ != -1) {
        close(serial_port_);
    }
}

void GPSParser::start_reading()
{
    std::thread([this]() {
        serial_port_ = open_serial_port(serial_device_.c_str(), baud_rate_);
        if (serial_port_ == -1) {
            std::cerr << "Open port failed." << std::endl;
            return;
        }

        std::cout << "Start reading GPS serial data (Press Ctrl+C to exit)..." << std::endl;
        std::string data;
        char buffer[GPS_PARSE_LEN + 1];
        while (true) {
            ssize_t buffer_len = read(serial_port_, buffer, GPS_PARSE_LEN);
            if (buffer_len < 0) {
                std::cerr << "Reading buffer failed.";
                return;
            }
            buffer[buffer_len] = '\0';
            data.append(buffer);

            size_t start, end;
            while ((start = data.find("$")) != std::string::npos) {
                end = data.find("\r\n", start);
                if (end == std::string::npos) {
                    break;
                }

                std::string msg = data.substr(start, end - start + 2);
                parse_msg(msg);
                data.erase(0, end + 2);
            }

            // 限制缓冲区长度，防止无限增长
            // if (data.size() > 1024) {
            //     data.erase(0, data.size() - 512);
            // } else {
            //     usleep(10000); // 防止CPU占用过高
            // }
        }
    }).detach();
}

std::optional<GPS_MSG_S> GPSParser::pop_msg()
{
    std::lock_guard<std::mutex> lock(msg_queue_mtx_);
    if (!msg_queue_.empty()) {
        GPS_MSG_S gps_msg = msg_queue_.front();
        msg_queue_.pop();
        return gps_msg;
    } 
    return std::nullopt;
}
