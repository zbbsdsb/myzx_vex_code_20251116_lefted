// KalmanFilter.h
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
private:
    // 状态向量: [x, y, heading, velocity, angular_velocity]
    double state[5];
    
    // 协方差矩阵 5x5
    double covariance[5][5];
    
    // 噪声参数
    double process_noise[5];
    double measurement_noise[3]; // 位置x,y和角度
    
public:
    KalmanFilter();
    
    // 初始化滤波器
    void init(double start_x, double start_y, double start_heading);
    
    // 预测步骤
    void predict(double left_speed, double right_speed, double dt, double wheel_distance);
    
    // 更新步骤
    void updateIMU(double measured_heading);
    void updatePosition(double measured_x, double measured_y);
    
    // 获取状态
    void getState(double &x, double &y, double &heading, double &velocity, double &angular_velocity);
    void getPosition(double &x, double &y, double &heading);
    
    // 设置位置 (用于重定位)
    void setPosition(double x, double y, double heading);
};

#endif