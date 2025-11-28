#include <iostream>
#include <iomanip>
#include <fstream>
#include <thread>
#include <queue>
#include <vector>
#include <optional>
#include <cmath>
#include <filesystem>
#include "imu_parser.h"
#include "gps_parser.h"
#include "ekfNavINS.h"

struct SENSOR_MSG_S {
    IMU_FRAME_S imu_frame;
    GPS_MSG_S gps_msg;
};

std::queue<SENSOR_MSG_S> g_sensor_msg_queue;
std::mutex g_sensor_msg_queue_mtx;

void push_sesor_msg(SENSOR_MSG_S sensor_msg)
{
    std::lock_guard<std::mutex> lock(g_sensor_msg_queue_mtx);
    g_sensor_msg_queue.push(sensor_msg);

    return;
}

std::optional<SENSOR_MSG_S> pop_sensor_msg()
{
    std::lock_guard<std::mutex> lock(g_sensor_msg_queue_mtx);
    if (!g_sensor_msg_queue.empty()) {
        SENSOR_MSG_S sensor_msg = g_sensor_msg_queue.front();
        g_sensor_msg_queue.pop();
        return sensor_msg;
    } 
    return std::nullopt;
}

struct EKF_MSG_S {
    double lat_ang;
    double lon_ang;
    double roll_ang;
    double pitch_ang;
    double heading_ang;
};

std::optional<std::ofstream> prepare_csv_file(const std::string& filename, const std::string& header) {
    const std::string log_dir = "log";
    namespace fs = std::filesystem;

    // 1. 确保 log 文件夹存在
    try {
        if (!fs::exists(log_dir)) {
            fs::create_directory(log_dir);
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "错误：创建 log 文件夹失败 - " << e.what() << std::endl;
        return std::nullopt;
    }

    // 2. 拼接完整路径
    std::string full_path = log_dir + "/" + filename;

    // 3. 打开文件（追加模式 + 输出模式）
    std::ofstream csv_file(full_path, std::ios::app | std::ios::out);
    if (!csv_file.is_open()) {
        std::cerr << "错误：无法打开文件 " << full_path << std::endl;  // 修复原代码：输出完整路径更直观
        return std::nullopt;
    }

    // 4. 文件为空时写入表头
    csv_file.seekp(0, std::ios::end);
    if (csv_file.tellp() == 0) {
        csv_file << header << std::endl;
    }

    // 5. 设置精度（公共精度配置，避免重复写）
    csv_file << std::setprecision(std::numeric_limits<double>::digits10 + 1);

    return csv_file;  // 返回打开的文件流
}

bool write_ekf_to_csv(const std::string& filename, const SENSOR_MSG_S& sensor_msg, const EKF_MSG_S& ekf_msg) {
    // 1. 定义当前业务的表头
    const std::string header = "frame_id,timestamp,time,"
                               "ekf_lat,ekf_lon,ekf_roll,ekf_pitch,ekf_heading";

    // 2. 调用工具函数准备文件
    auto csv_file_opt = prepare_csv_file(filename, header);
    if (!csv_file_opt.has_value()) {
        return false;  // 准备失败，直接返回
    }
    auto& csv_file = csv_file_opt.value();  // 获取文件流引用

    // 3. 仅写入当前业务的数据（核心逻辑）
    csv_file << sensor_msg.imu_frame.frame_id << ","
             << sensor_msg.imu_frame.time.ts_us << ","
             << sensor_msg.imu_frame.time.hour << ":"
             << sensor_msg.imu_frame.time.min << ":"
             << sensor_msg.imu_frame.time.sec << ":"
             << sensor_msg.imu_frame.time.ms << ","
             << ekf_msg.lat_ang << ","
             << ekf_msg.lon_ang << ","
             << ekf_msg.roll_ang << ","
             << ekf_msg.pitch_ang << ","
             << ekf_msg.heading_ang
             << std::endl;

    // 4. 刷新并自动关闭（文件流析构时会关闭，手动 flush 确保数据写入）
    csv_file.flush();
    return true;
}

bool write_imu_to_csv(const std::string& filename, const SENSOR_MSG_S& sensor_msg) {
    // 1. 定义当前业务的表头
    const std::string header = "time,"
                               "ax,ay,az,"
                               "wx,wy,wz,"
                               "hx,hy,hz";

    // 2. 调用工具函数准备文件
    auto csv_file_opt = prepare_csv_file(filename, header);
    if (!csv_file_opt.has_value()) {
        return false;  // 准备失败，直接返回
    }
    auto& csv_file = csv_file_opt.value();  // 获取文件流引用

    // 3. 仅写入当前业务的数据（核心逻辑）
    csv_file << sensor_msg.imu_frame.time.ts_us << ","
             << sensor_msg.imu_frame.acc.x << ","
             << sensor_msg.imu_frame.acc.y << ","
             << sensor_msg.imu_frame.acc.z << ","
             << sensor_msg.imu_frame.gyro.x << ","
             << sensor_msg.imu_frame.gyro.y << ","
             << sensor_msg.imu_frame.gyro.z << ","
             << sensor_msg.imu_frame.mag.x << ","
             << sensor_msg.imu_frame.mag.y << ","
             << sensor_msg.imu_frame.mag.z
             << std::endl;

    // 4. 刷新并自动关闭（文件流析构时会关闭，手动 flush 确保数据写入）
    csv_file.flush();
    return true;
}

bool write_gps_to_csv(const std::string& filename, const GPS_MSG_S& gps_msg) {
    // 1. 定义当前业务的表头
    const std::string header = "time,lat,lon,msl,kph,cogt";

    // 2. 调用工具函数准备文件
    auto csv_file_opt = prepare_csv_file(filename, header);
    if (!csv_file_opt.has_value()) {
        return false;  // 准备失败，直接返回
    }
    auto& csv_file = csv_file_opt.value();  // 获取文件流引用

    // 3. 仅写入当前业务的数据（核心逻辑）
    csv_file << gps_msg.time.ts_us << ","
             << gps_msg.gngga.lat_ang << ","
             << gps_msg.gngga.lon_ang << ","
             << gps_msg.gngga.msl << ","
             << gps_msg.gnvtg.kph << ","
             << gps_msg.gnvtg.cogt_ang
             << std::endl;


    // 4. 刷新并自动关闭（文件流析构时会关闭，手动 flush 确保数据写入）
    csv_file.flush();
    return true;
}

void ekf_fusion()
{
    // 1. 获取当前系统时间（时间戳，精确到秒）
    auto now = std::chrono::system_clock::now();
    // 2. 转换为 time_t 类型（兼容传统时间函数）
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    // 3. 转换为本地时间（避免UTC时差问题）
    std::tm local_tm = *std::localtime(&now_time);  // localtime 线程不安全，下文有优化方案
    // 4. 格式化字符串（拼接文件名）
    std::stringstream ekf_filename;
    ekf_filename << "ekf_"
                 << std::put_time(&local_tm, "%Y_%m_%d_%H_%M_%S")  // 核心格式化：年_月_日_时_分_秒
                 << ".csv";
    std::stringstream imu_filename;
    imu_filename << "imu_"
                 << std::put_time(&local_tm, "%Y_%m_%d_%H_%M_%S")  // 核心格式化：年_月_日_时_分_秒
                 << ".csv";

    ekfNavINS ekf;
    while (true) {
        if (auto sensor_msg = pop_sensor_msg()) {
            if (false) {
                std::cout << "--------------------------------------------------" << std::endl;
                std::cout << "msg_id: " << sensor_msg->gps_msg.msg_id << ". ";
                std::cout << "timestamp_us: " << sensor_msg->gps_msg.time.ts_us << std::endl;
                std::cout << "time: " << sensor_msg->gps_msg.time.hour << ":" << sensor_msg->gps_msg.time.min << ":" << sensor_msg->gps_msg.time.sec << "." << sensor_msg->gps_msg.time.ms << std::endl;
                // std::cout << "lat_ang: " << sensor_msg->gps_msg.gngga.lat_ang << std::endl;
                // std::cout << "lat_rad: " << sensor_msg->gps_msg.gngga.lat_rad << std::endl;
                // std::cout << "lon_ang: " << sensor_msg->gps_msg.gngga.lon_ang << std::endl;
                // std::cout << "lon_rad: " << sensor_msg->gps_msg.gngga.lon_rad << std::endl;
                // std::cout << "msl: " << sensor_msg->gps_msg.gngga.msl << std::endl;
                // std::cout << "kph: " << sensor_msg->gps_msg.gnvtg.kph << std::endl;
                // std::cout << "kph_east: " << sensor_msg->gps_msg.gnvtg.kph_east << std::endl;
                // std::cout << "kph_north: " << sensor_msg->gps_msg.gnvtg.kph_north << std::endl;
                // std::cout << "kph_up: " << sensor_msg->gps_msg.gnvtg.kph_up << std::endl;
                // std::cout << "cogt_ang: " << sensor_msg->gps_msg.gnvtg.cogt_ang << std::endl;
                // std::cout << "cogt_rad: " << sensor_msg->gps_msg.gnvtg.cogt_rad << std::endl;

                std::cout << "frame_id: " << sensor_msg->imu_frame.frame_id << ". ";
                std::cout << "timestamp_us: " << sensor_msg->imu_frame.time.ts_us << std::endl;
                std::cout << sensor_msg->imu_frame.time.hour << ":" << sensor_msg->imu_frame.time.min << ":" << sensor_msg->imu_frame.time.sec << ":" << sensor_msg->imu_frame.time.ms << " ";
                std::cout << "[acc_x,   acc_y,   acc_z  ]: " << sensor_msg->imu_frame.acc.x << ", " << sensor_msg->imu_frame.acc.y << ", " << sensor_msg->imu_frame.acc.z << ". ";
                std::cout << "[gyro_x,  gyro_y,  gyro_z ]: " << sensor_msg->imu_frame.gyro.x << ", " << sensor_msg->imu_frame.gyro.y << ", " << sensor_msg->imu_frame.gyro.z << ". ";
                std::cout << "[angle_x, angle_y, angle_z]: " << sensor_msg->imu_frame.angle.x << ", " << sensor_msg->imu_frame.angle.y << ", " << sensor_msg->imu_frame.angle.z << ". ";
                // std::cout << "[mag_x,   mag_y,   mag_z  ]: " << sensor_msg->imu_frame.mag.x << ", " << sensor_msg->imu_frame.mag.y << ", " << sensor_msg->imu_frame.mag.z << std::endl;
                std::cout << std::endl;
            } else {
                ekf.ekf_update(sensor_msg->imu_frame.time.ts_us,
                    sensor_msg->gps_msg.gnvtg.kph_north/3.6, sensor_msg->gps_msg.gnvtg.kph_east/3.6, -1*sensor_msg->gps_msg.gnvtg.kph_up/3.6,
                    sensor_msg->gps_msg.gngga.lat_rad, sensor_msg->gps_msg.gngga.lon_rad, sensor_msg->gps_msg.gngga.msl,
                    sensor_msg->imu_frame.gyro.x*M_PI/180.0, sensor_msg->imu_frame.gyro.y*M_PI/180.0, sensor_msg->imu_frame.gyro.z*M_PI/180.0,
                    sensor_msg->imu_frame.acc.x*9.794, sensor_msg->imu_frame.acc.y*9.794, sensor_msg->imu_frame.acc.z*9.794,
                    sensor_msg->imu_frame.mag.x*1e-3, sensor_msg->imu_frame.mag.y*1e-3, sensor_msg->imu_frame.mag.z*1e-3);
                std::cout << "--------------------------------------------------" << std::endl;
                std::cout << std::dec << "frame_id: " << sensor_msg->imu_frame.frame_id << ". ";
                std::cout << "timestamp_us: " << sensor_msg->imu_frame.time.ts_us << std::endl;
                std::cout << "time: " << sensor_msg->imu_frame.time.hour << ":" << sensor_msg->imu_frame.time.min << ":" << sensor_msg->imu_frame.time.sec << "." << sensor_msg->imu_frame.time.ms << std::endl;
                std::cout << "gps_lat: " << sensor_msg->gps_msg.gngga.lat_ang << std::endl;
                std::cout << "gps_lon: " << sensor_msg->gps_msg.gngga.lon_ang << std::endl;
                std::cout << "ekf_lat : " << ekf.getLatitude_rad()*(180.0/M_PI) << std::endl;
                std::cout << "ekf_lon: " << ekf.getLongitude_rad()*(180.0/M_PI) << std::endl;

                EKF_MSG_S ekf_msg;
                ekf_msg.lat_ang = ekf.getLatitude_rad()*(180.0/M_PI);
                ekf_msg.lon_ang = ekf.getLongitude_rad()*(180.0/M_PI);
                ekf_msg.roll_ang = ekf.getRoll_rad()*(180.0/M_PI);
                ekf_msg.pitch_ang = ekf.getPitch_rad()*(180.0/M_PI);
                ekf_msg.heading_ang = ekf.getHeading_rad()*(180.0/M_PI);

                write_ekf_to_csv(ekf_filename.str(), *sensor_msg, ekf_msg);
                write_imu_to_csv(imu_filename.str(), *sensor_msg);
            }
        }
    }
}

GPS_MSG_S gps_linear_interpolate(GPS_MSG_S &prev, GPS_MSG_S &curr, uint64_t inter_time)
{
    if (curr.time.ts_us == prev.time.ts_us) {
        return curr;
    }

    uint64_t delta_t = curr.time.ts_us - prev.time.ts_us;
    double ratio = (inter_time - prev.time.ts_us) / delta_t;

    GPS_MSG_S inter;

    inter.gngga.lat_rad = prev.gngga.lat_rad + ratio * (curr.gngga.lat_rad - prev.gngga.lat_rad);
    inter.gngga.lat_ang = prev.gngga.lat_ang + ratio * (curr.gngga.lat_ang - prev.gngga.lat_ang);
    inter.gngga.lon_rad = prev.gngga.lon_rad + ratio * (curr.gngga.lon_rad - prev.gngga.lon_rad);
    inter.gngga.lon_ang = prev.gngga.lon_ang + ratio * (curr.gngga.lon_ang - prev.gngga.lon_ang);
    inter.gngga.msl = prev.gngga.msl + ratio * (curr.gngga.msl - prev.gngga.msl);

    inter.gnvtg.kph = prev.gnvtg.kph + ratio * (curr.gnvtg.kph - prev.gnvtg.kph);
    inter.gnvtg.kph_east = prev.gnvtg.kph_east + ratio * (curr.gnvtg.kph_east - prev.gnvtg.kph_east);
    inter.gnvtg.kph_north = prev.gnvtg.kph_north + ratio * (curr.gnvtg.kph_north - prev.gnvtg.kph_north);
    inter.gnvtg.kph_up = prev.gnvtg.kph_up + ratio * (curr.gnvtg.kph_up - prev.gnvtg.kph_up);
    inter.gnvtg.cogt_ang = prev.gnvtg.cogt_ang + ratio * (curr.gnvtg.cogt_ang - prev.gnvtg.cogt_ang);
    inter.gnvtg.cogt_rad = prev.gnvtg.cogt_rad + ratio * (curr.gnvtg.cogt_rad - prev.gnvtg.cogt_rad);

    inter.time.ts_us = prev.time.ts_us;
    inter.time.hour = prev.time.hour;
    inter.time.min = prev.time.min;
    inter.time.sec = prev.time.sec;
    inter.time.ms = prev.time.ms;

    inter.msg_id = prev.msg_id;

    return inter;
}

// Preprocessing & Time Sync
void gps_imu_sensor_fusion(GPSParser& gps_parser, IMUParser& imu_parser)
{
    // 1. 获取当前系统时间（时间戳，精确到秒）
    auto now = std::chrono::system_clock::now();
    // 2. 转换为 time_t 类型（兼容传统时间函数）
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    // 3. 转换为本地时间（避免UTC时差问题）
    std::tm local_tm = *std::localtime(&now_time);  // localtime 线程不安全，下文有优化方案
    // 4. 格式化字符串（拼接文件名）
    std::stringstream gps_filename;
    gps_filename << "gps_"
                 << std::put_time(&local_tm, "%Y_%m_%d_%H_%M_%S")  // 核心格式化：年_月_日_时_分_秒
                 << ".csv";

    // 过滤启动之后GPS和IMU不同步的数据
    GPS_MSG_S prev_gps_msg = [&]() { std::optional<GPS_MSG_S> opt = gps_parser.pop_msg(); while (!opt.has_value()) { opt = gps_parser.pop_msg(); } return opt.value(); }();
    GPS_MSG_S curr_gps_msg = [&]() { std::optional<GPS_MSG_S> opt = gps_parser.pop_msg(); while (!opt.has_value()) { opt = gps_parser.pop_msg(); } return opt.value(); }();
    IMU_FRAME_S imu_frame = [&]() { std::optional<IMU_FRAME_S> opt = imu_parser.pop_frame(); while (!opt.has_value()) { opt = imu_parser.pop_frame(); } return opt.value(); }();

    while (prev_gps_msg.time.ts_us > imu_frame.time.ts_us) {
        imu_frame = [&]() { std::optional<IMU_FRAME_S> opt = imu_parser.pop_frame(); while (!opt.has_value()) { opt = imu_parser.pop_frame(); } return opt.value(); }();
    }

    while (curr_gps_msg.time.ts_us < imu_frame.time.ts_us) {
        prev_gps_msg = curr_gps_msg;
        curr_gps_msg = [&]() { std::optional<GPS_MSG_S> opt = gps_parser.pop_msg(); while (!opt.has_value()) { opt = gps_parser.pop_msg(); } return opt.value(); }();
    }

    SENSOR_MSG_S sensor_msg;
    sensor_msg.gps_msg = gps_linear_interpolate(prev_gps_msg, curr_gps_msg, imu_frame.time.ts_us);
    sensor_msg.imu_frame = imu_frame;
    push_sesor_msg(sensor_msg);

    // 一次循环只弹出一个IMU
    while (true) {  
        imu_frame = [&]() { std::optional<IMU_FRAME_S> opt = imu_parser.pop_frame(); while (!opt.has_value()) { opt = imu_parser.pop_frame(); } return opt.value(); }();

        while (curr_gps_msg.time.ts_us < imu_frame.time.ts_us) {
            prev_gps_msg = curr_gps_msg;
            curr_gps_msg = [&]() { std::optional<GPS_MSG_S> opt = gps_parser.pop_msg(); while (!opt.has_value()) { opt = gps_parser.pop_msg(); } return opt.value(); }();
            write_gps_to_csv(gps_filename.str(), curr_gps_msg);
        }

        SENSOR_MSG_S sensor_msg;
        sensor_msg.gps_msg = gps_linear_interpolate(prev_gps_msg, curr_gps_msg, imu_frame.time.ts_us);
        sensor_msg.imu_frame = imu_frame;
        push_sesor_msg(sensor_msg);
    }
}

void get_gps2_msg(GPSParser& gps_parser)
{
    // 1. 获取当前系统时间（时间戳，精确到秒）
    auto now = std::chrono::system_clock::now();
    // 2. 转换为 time_t 类型（兼容传统时间函数）
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    // 3. 转换为本地时间（避免UTC时差问题）
    std::tm local_tm = *std::localtime(&now_time);  // localtime 线程不安全，下文有优化方案
    // 4. 格式化字符串（拼接文件名）
    std::stringstream gps_filename;
    gps_filename << "gps_2_"
                 << std::put_time(&local_tm, "%Y_%m_%d_%H_%M_%S")  // 核心格式化：年_月_日_时_分_秒
                 << ".csv";

    while (true) {
        GPS_MSG_S gps_msg = [&]() { std::optional<GPS_MSG_S> opt = gps_parser.pop_msg(); while (!opt.has_value()) { opt = gps_parser.pop_msg(); std::this_thread::sleep_for(std::chrono::milliseconds(100)); } return opt.value(); }();
        write_gps_to_csv(gps_filename.str(), gps_msg);
    }
}

int main()
{
    IMUParser imu_parser("/dev/ttyUSB0", B115200);
    GPSParser gps_parser_1("/dev/ttyUSB1", B115200);
    GPSParser gps_parser_2("/dev/ttyUSB2", B9600);

    imu_parser.start_reading();
    gps_parser_1.start_reading();
    gps_parser_2.start_reading();

    std::thread gps_imu_sensor_fusion_thread(gps_imu_sensor_fusion, std::ref(gps_parser_1), std::ref(imu_parser));
    std::thread ekf_fusion_thread(ekf_fusion);
    std::thread get_gps2_msg_thread(get_gps2_msg, std::ref(gps_parser_2));

    if (gps_imu_sensor_fusion_thread.joinable()) {
        gps_imu_sensor_fusion_thread.join();
    }
    if (ekf_fusion_thread.joinable()) {
        ekf_fusion_thread.join();
    }

    if (get_gps2_msg_thread.joinable()) {
        get_gps2_msg_thread.join();
    }

    return 0;
}
