#include "vex.h"
#include "Move_Motor.h"
using namespace vex;

void EncoderInit()
{
  Left_Motor1.setMaxTorque(100,percent);
  Left_Motor2.setMaxTorque(100,percent);
  Left_Motor3.setMaxTorque(100,percent);
  Right_Motor1.setMaxTorque(100,percent);
  Right_Motor2.setMaxTorque(100,percent);
  Right_Motor3.setMaxTorque(100,percent);
  SHAI.setMaxTorque(100,percent);
  DINGBU.setMaxTorque(100,percent);
  
  A.set(false); 
  B.set(false);
}

void EncoderReset()
{
  Left_Motor1.resetPosition();
  Left_Motor2.resetPosition();
  Left_Motor3.resetPosition();
  Right_Motor1.resetPosition();
  Right_Motor2.resetPosition();
  Right_Motor3.resetPosition();
  SHAI.resetPosition();
  DINGBU.resetPosition();
  
}

void Forward_Degree(int left_power,int right_power,int degree){
  EncoderReset();
  Move(left_power,right_power);
  waitUntil((Left_Motor1.position(degrees) > degree));
  Move_Stop_brake();
  wait(20, msec);
}

void Reverse_Degree(int left_power,int right_power,int degree){
  EncoderReset();
  Move_Reverse(left_power,right_power);
  waitUntil((Left_Motor1.position(degrees) < degree));
  Move_Stop_brake();
  wait(20, msec);
}

void PID_Turn(int degree){
  // 定义 PID 控制器的参数
double kP = 0.23; // 比例系数
double kI = 0.12; // 积分系数
double kD = 0.05; // 微分系数

// 定义 PID 控制器的状态
double error = 0;
double integral = 0;
double derivative = 0;

while(1){
        // 读取当前角度
        double currentAngle = imu.rotation();
        // 计算角度偏差
        error = degree-currentAngle;
        // 计算积分项
        integral = integral/2 + error;

        // 计算微分项
        derivative = error - (degree - imu.rotation());
        // 计算 PID 控制输出
        double PID_out = kP * error + kI * integral + kD * derivative;

 Move_Forward(PID_out,-PID_out);
 if(fabs(error)<11){
 Move_Stop_brake();
 wait(20, msec);
    break;
    }
  }
}

void PID_Move(int left_power,int right_power,int degree,int timelimit){
  // 定义 PID 控制器的参数
double kP = 0.23; // 比例系数
double kI = 0.12; // 积分系数
double kD = 0.05; // 微分系数

// 定义 PID 控制器的状态
double error = 0;
double integral = 0;
double derivative = 0;
EncoderReset();
Brain.resetTimer();

while(1){
        // 读取当前角度
        double currentAngle = Left_Motor1.position(degrees);
        // 计算角度偏差
        error = degree-currentAngle;
        // 计算积分项
        integral = integral/2 + error;

        // 计算微分项
        derivative = error - (degree - (Left_Motor1.position(degrees)));
        // 计算 PID 控制输出
        double PID_out = kP * error + kI * integral + kD * derivative;

 Move(PID_out*left_power/100,PID_out*right_power/100);
 if(fabs(error)<10 || Brain.timer(timeUnits::msec)>timelimit){
    Move_Stop_brake();
    wait(20, msec);
    break;
    }
  }

}

