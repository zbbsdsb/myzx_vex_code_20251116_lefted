// KalmanFilter.cpp
#include "KalmanFilter.h"
#include <cmath>

KalmanFilter::KalmanFilter() {
    // 初始化状态和协方差
    for(int i = 0; i < 5; i++) {
        state[i] = 0.0;
        for(int j = 0; j < 5; j++) {
            covariance[i][j] = 0.0;
        }
    }
    
    // 设置噪声参数
    process_noise[0] = 0.01; // x位置噪声
    process_noise[1] = 0.01; // y位置噪声  
    process_noise[2] = 0.005; // 角度噪声
    process_noise[3] = 0.1;  // 速度噪声
    process_noise[4] = 0.05; // 角速度噪声
    
    measurement_noise[0] = 0.02; // x测量噪声
    measurement_noise[1] = 0.02; // y测量噪声
    measurement_noise[2] = 0.01; // 角度测量噪声
}

void KalmanFilter::init(double start_x, double start_y, double start_heading) {
    state[0] = start_x;     // x
    state[1] = start_y;     // y
    state[2] = start_heading; // heading
    state[3] = 0.0;         // velocity
    state[4] = 0.0;         // angular_velocity
    
    // 初始化协方差 - 较大的不确定性
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < 5; j++) {
            covariance[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

void KalmanFilter::predict(double left_speed, double right_speed, double dt, double track_width) {
    if(dt <= 0) return;
    
    // 差速驱动运动模型
    double v = (left_speed + right_speed) / 2.0;
    double omega = (right_speed - left_speed) / track_width;
    
    double theta = state[2];
    
    // 状态预测
    if(fabs(omega) < 0.001) {
        // 近似直线运动
        state[0] += v * cos(theta) * dt;
        state[1] += v * sin(theta) * dt;
    } else {
        // 精确圆弧运动
        double delta_theta = omega * dt;
        state[0] += (v/omega) * (sin(theta + delta_theta) - sin(theta));
        state[1] += (v/omega) * (cos(theta) - cos(theta + delta_theta));
        state[2] += delta_theta;
    }
    
    state[3] = v;
    state[4] = omega;
    
    // 标准化角度到 [-PI, PI]
    while(state[2] > M_PI) state[2] -= 2 * M_PI;
    while(state[2] < -M_PI) state[2] += 2 * M_PI;
    
    // 简化协方差预测 - 添加过程噪声
    for(int i = 0; i < 5; i++) {
        covariance[i][i] += process_noise[i] * dt;
    }
}

void KalmanFilter::updateIMU(double measured_heading) {
    // 角度残差 (考虑角度环绕)
    double angle_error = measured_heading - state[2];
    while(angle_error > M_PI) angle_error -= 2 * M_PI;
    while(angle_error < -M_PI) angle_error += 2 * M_PI;
    
    // 卡尔曼增益 (只更新角度)
    double S = covariance[2][2] + measurement_noise[2];
    double K[5];
    for(int i = 0; i < 5; i++) {
        K[i] = covariance[i][2] / S;
    }
    
    // 状态更新
    for(int i = 0; i < 5; i++) {
        state[i] += K[i] * angle_error;
    }
    
    // 协方差更新
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < 5; j++) {
            covariance[i][j] -= K[i] * covariance[2][j];
        }
    }
}

void KalmanFilter::updatePosition(double measured_x, double measured_y) {
    // 位置残差
    double pos_error_x = measured_x - state[0];
    double pos_error_y = measured_y - state[1];
    
    // 简化更新 - 直接融合
    state[0] += 0.5 * pos_error_x;
    state[1] += 0.5 * pos_error_y;
    
    // 增加位置不确定性
    covariance[0][0] *= 0.8;
    covariance[1][1] *= 0.8;
}

void KalmanFilter::getState(double &x, double &y, double &heading, double &velocity, double &angular_velocity) {
    x = state[0];
    y = state[1];
    heading = state[2];
    velocity = state[3];
    angular_velocity = state[4];
}

void KalmanFilter::getPosition(double &x, double &y, double &heading) {
    x = state[0];
    y = state[1];
    heading = state[2];
}

void KalmanFilter::setPosition(double x, double y, double heading) {
    state[0] = x;
    state[1] = y;
    state[2] = heading;
    
    // 重置位置不确定性
    covariance[0][0] = 0.01;
    covariance[1][1] = 0.01;
    covariance[2][2] = 0.01;
}