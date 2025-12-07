// Localization.cpp
#include "Localization.h"
#include "vex.h"
#include <cmath>

using namespace vex;

LocalizationSystem localization;

LocalizationSystem::LocalizationSystem() {
    initialized = false;
    last_update_time = 0;
    left_encoder_prev = right_encoder_prev = 0;
    
    // 机器人参数 - 需要根据实际情况调整
    wheel_circumference = 12.56; // 4英寸直径轮子的周长
    track_width = 11.0;          // 轮距 (英寸)
}

void LocalizationSystem::init(double start_x, double start_y, double start_heading) {
    // 转换为弧度
    double heading_rad = start_heading * M_PI / 180.0;
    
    kf.init(start_x, start_y, heading_rad);
    
    // 重置编码器
    Left_Motor1.resetPosition();
    Right_Motor1.resetPosition();
    left_encoder_prev = 0;
    right_encoder_prev = 0;
    
    last_update_time = Brain.timer(msec);
    initialized = true;
    
    printf("定位系统初始化: (%.1f, %.1f, %.1f°)\n", start_x, start_y, start_heading);
}

void LocalizationSystem::update() {
    if(!initialized) return;
    
    double current_time = Brain.timer(msec);
    double dt = (current_time - last_update_time) / 1000.0; // 转换为秒
    
    if(dt <= 0 || dt > 0.1) {
        last_update_time = current_time;
        return;
    }
    
    // 获取编码器读数 (度)
    double left_deg = Left_Motor1.position(degrees);
    double right_deg = Right_Motor1.position(degrees);
    
    // 计算轮子线速度 (英寸/秒)
    double left_speed = ((left_deg - left_encoder_prev) / 360.0) * wheel_circumference / dt;
    double right_speed = ((right_deg - right_encoder_prev) / 360.0) * wheel_circumference / dt;
    
    // 更新前值
    left_encoder_prev = left_deg;
    right_encoder_prev = right_deg;
    last_update_time = current_time;
    
    // 卡尔曼滤波预测
    kf.predict(left_speed, right_speed, dt, track_width);
    
    // 使用IMU更新
    double imu_heading = imu.rotation(degrees) * M_PI / 180.0; // 转换为弧度
    kf.updateIMU(imu_heading);
}

void LocalizationSystem::getPosition(double &x, double &y, double &heading) {
    double v, omega;
    kf.getState(x, y, heading, v, omega);
    heading = heading * 180.0 / M_PI; // 转换为度
}

void LocalizationSystem::getFullState(double &x, double &y, double &heading, double &velocity, double &angular_velocity) {
    kf.getState(x, y, heading, velocity, angular_velocity);
    heading = heading * 180.0 / M_PI; // 转换为度
}

void LocalizationSystem::setPosition(double x, double y, double heading) {
    double heading_rad = heading * M_PI / 180.0;
    kf.setPosition(x, y, heading_rad);
    printf("重定位到: (%.1f, %.1f, %.1f°)\n", x, y, heading);
}