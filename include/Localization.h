// Localization.h
#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include "KalmanFilter.h"

// 全局定位系统
class LocalizationSystem {
private:
    KalmanFilter kf;
    double last_update_time;
    double left_encoder_prev, right_encoder_prev;
    bool initialized;
    
    // 机器人参数
    double wheel_circumference;  // 轮子周长 (英寸)
    double track_width;          // 轮距 (英寸)
    
public:
    LocalizationSystem();
    
    // 初始化系统
    void init(double start_x = 0, double start_y = 0, double start_heading = 0);
    
    // 更新定位 (主循环调用)
    void update();
    
    // 获取位置
    void getPosition(double &x, double &y, double &heading);
    void getFullState(double &x, double &y, double &heading, double &velocity, double &angular_velocity);
    
    // 设置位置 (重定位)
    void setPosition(double x, double y, double heading);
    
    // 状态检查
    bool isInitialized() { return initialized; }
};

// 全局实例
extern LocalizationSystem localization;

#endif