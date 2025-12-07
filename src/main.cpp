/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       myzx_ceaserzhao                                                       */
/*    Created:      Tue Nov 18 2025                                           */
/*    Description:  for myzx vex club<3                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Left_Motor1          motor         1               
// Left_Motor2          motor         2               
// Left_Motor3          motor         3               
// Right_Motor1         motor         10              
// Right_Motor2         motor         9               
// Right_Motor3         motor         8               
// SHAI                 motor         7               
// DINGBU               motor         11              
// imu                  inertial      12              
// A                    digital_out   A               
// B                    digital_out   B               
// Rotation19           rotation      19              
// Rotation20           rotation      20              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "PID.h"
#include "Move_Motor.h"
#include "Auxiliary.h"
#include "Auto.h"
#include "autons.h"
#include "Localization.h"
#include "FieldMap.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

// 定位系统状态
bool localization_enabled = true;
bool localization_initialized = false;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) 
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // 初始化编码器和电机
  EncoderReset();
  EncoderInit();
  
  // 显示初始化信息
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Initializing Robot...");
  
  // IMU 校准
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Calibrating IMU...");
  printf("开始IMU校准...\n");
  
  imu.calibrate();
  
  // 等待校准完成
  while (imu.isCalibrating()) {
    wait(100, msec);
  }
  
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("IMU Calibration Complete");
  printf("IMU校准完成\n");
  
  // 初始化定位系统
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Initializing Localization...");
  
  // 根据比赛起始位置设置初始位置
  // 这里假设从场地左下角开始，朝向0度(正上方)
  localization.init(12.0, 12.0, 0.0);
  localization_initialized = true;
  
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("Localization Ready");
  
  // 显示完成信息
  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("=== System Ready ===");
  Brain.Screen.setCursor(7, 1);
  Brain.Screen.print("Press A for Auto");
  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("Press B for Driver");
  
  printf("=== 机器人系统初始化完成 ===\n");
  printf("定位系统已启动 - 初始位置: (12.0, 12.0, 0.0°)\n");
  
  // 初始显示地图
  double x, y, heading;
  localization.getPosition(x, y, heading);
  FieldMap::display(x, y, heading);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) 
{
  printf("=== 开始自动程序 ===\n");
  
  // 显示自动模式信息
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.setPenColor(green);
  Brain.Screen.print("AUTONOMOUS MODE");
  Brain.Screen.setPenColor(white);
  
  // 启动定位更新线程
  thread localization_thread = thread([]() {
    printf("定位线程启动\n");
    int update_count = 0;
    
    while(localization_enabled) {
      if (localization_initialized) {
        // 更新定位
        localization.update();
        
        // 每20次更新显示一次调试信息（避免过于频繁）
        if(update_count % 20 == 0) {
          double x, y, heading, velocity, angular_velocity;
          localization.getFullState(x, y, heading, velocity, angular_velocity);
          
          // 在调试控制台输出位置信息
          printf("Auto - 位置: (%.1f, %.1f, %.1f°) 速度: %.1f\n", 
                 x, y, heading, velocity);
        }
        
        // 实时更新屏幕显示
        double x, y, heading;
        localization.getPosition(x, y, heading);
        FieldMap::display(x, y, heading);
        
        update_count++;
      }
      wait(50, msec); // 20Hz 更新频率
    }
    printf("自动模式定位线程结束\n");
  });

  // 执行自动程序
  try {
    // 显示执行信息
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Running Auto Program...");
    
    // 调用你的自动程序
    RunAuto();
    
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.setPenColor(green);
    Brain.Screen.print("Auto Program Complete");
    Brain.Screen.setPenColor(white);
    
    printf("自动程序执行完成\n");
    
  } catch (...) {
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Auto Program Error!");
    Brain.Screen.setPenColor(white);
    
    printf("自动程序执行出错\n");
  }
  
  // 显示最终位置
  if (localization_initialized) {
    double final_x, final_y, final_heading;
    localization.getPosition(final_x, final_y, final_heading);
    
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("Final Position:");
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("X: %.1f, Y: %.1f", final_x, final_y);
    Brain.Screen.setCursor(9, 1);
    Brain.Screen.print("Heading: %.1f°", final_heading);
    
    printf("最终位置: (%.1f, %.1f, %.1f°)\n", final_x, final_y, final_heading);
  }
  
  // 保持程序运行直到自动阶段结束
  while(Competition.isAutonomous()) {
    wait(100, msec);
  }
  
  printf("自动阶段结束\n");
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void thread_Move()
{
  int Axis_1,Axis_2,Axis_3,Axis_4;
  bool Button_R1,Button_R2;
  bool Button_L1,Button_L2;
  
  while (1) 
  {
    Axis_1=Controller1.Axis1.value();
    Axis_2=Controller1.Axis2.value();
    Axis_3=Controller1.Axis3.value();
    Axis_4=Controller1.Axis4.value();

    Button_R1=Controller1.ButtonR1.pressing();
    Button_R2=Controller1.ButtonR2.pressing();
    Button_L1=Controller1.ButtonL1.pressing();
    Button_L2=Controller1.ButtonL2.pressing();
    
    //====================================================手柄定义

    // 底盘控制逻辑
    if (abs(Axis_3)<5 && abs(Axis_1)>5)
    {
      // 原地转向
      Move(0.7 * Axis_1,- 0.7 * Axis_1);
    }
    else if (abs(Axis_3)>5)
    {
      // 弧线移动
      Move(Axis_3 + 0.7 * Axis_1,Axis_3 - 0.7 * Axis_1);
    }
    else
    {
      // 停止
      Move_Stop_brake();
    }

    // 滚筒控制
    if (Button_R1)
    {
     SHAIControl(-100);
    }
    else if (Button_R2) 
    {
      SHAIControl(100);
    }
    else 
    {
      SHAIControl(0);
    }
    
    // 顶部装置控制
    if (Button_L1)
    {
     DINGBUControl(100);
    }
    else if (Button_L2) 
    {
      DINGBUControl(-100);
    }
    else 
    {
      DINGBUControl(0);
    }
    
    wait(20, msec);
  }
}

void drivercontrol(void) 
{
  printf("=== 进入手动控制模式 ===\n");
  
  // 显示手动模式信息
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.setPenColor(blue);
  Brain.Screen.print("DRIVER CONTROL MODE");
  Brain.Screen.setPenColor(white);
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("X: Reset Position");
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Y: Set to Center");
  
  int f = 0;
  int g = 0;
  thread task1, localization_task;
  bool Button_A,Button_B,Button_Y,Button_X;
  bool Button_L1,Button_L2,Button_R1,Button_R2;
  bool Button_Up,Button_Down,Button_Left,Button_Right;
  
  EncoderReset();
  
  // 启动定位更新线程
  localization_task = thread([]() {
    printf("手动模式定位线程启动\n");
    int update_count = 0;
    
    while(localization_enabled) {
      if (localization_initialized) {
        // 更新定位
        localization.update();
        
        // 每30次更新显示一次调试信息
        if(update_count % 30 == 0) {
          double x, y, heading, velocity, angular_velocity;
          localization.getFullState(x, y, heading, velocity, angular_velocity);
          printf("Driver - 位置: (%.1f, %.1f, %.1f°) 速度: %.1f\n", 
                 x, y, heading, velocity);
        }
        
        // 实时更新屏幕显示
        double x, y, heading;
        localization.getPosition(x, y, heading);
        FieldMap::display(x, y, heading);
        
        update_count++;
      }
      wait(50, msec); // 20Hz 更新频率
    }
    printf("手动模式定位线程结束\n");
  });

  // 启动底盘控制线程
  task1 = thread(thread_Move);
  
  // 主控制循环
  while (1) 
  {
    // 读取手柄按钮
    Button_A=Controller1.ButtonA.pressing();
    Button_B=Controller1.ButtonB.pressing();
    Button_X=Controller1.ButtonX.pressing();
    Button_Y=Controller1.ButtonY.pressing();
    
    Button_L1=Controller1.ButtonL1.pressing();    
    Button_L2=Controller1.ButtonL2.pressing();
    Button_R1=Controller1.ButtonR1.pressing();    
    Button_R2=Controller1.ButtonR2.pressing();

    Button_Up=Controller1.ButtonUp.pressing();
    Button_Down=Controller1.ButtonDown.pressing();
    Button_Left=Controller1.ButtonLeft.pressing();
    Button_Right=Controller1.ButtonRight.pressing();

    // 气动装置控制逻辑
    if(Button_A && f == 0){
      A.set(1);
      waitUntil(!Controller1.ButtonA.pressing());
      f = 1;
    }
    else if(Button_A && f == 1){
      A.set(0);
      waitUntil(!Controller1.ButtonA.pressing());
      f = 0;
    }
    else if(f == 0){
      A.set(0);
    }
    
    if(Button_B && g == 0){
      B.set(1);
      waitUntil(!Controller1.ButtonB.pressing());
      g = 1;
    }
    else if(Button_B && g == 1){
      B.set(0);
      waitUntil(!Controller1.ButtonB.pressing());
      g = 0;
    }
    else if(g == 0){
      B.set(0);
    }
    
    // 重定位功能
    if(Button_X) {
      // 重置到初始位置
      localization.setPosition(12.0, 12.0, 0.0);
      Brain.Screen.setCursor(5, 1);
      Brain.Screen.print("Position Reset to Start");
      printf("已重定位到起始位置\n");
      waitUntil(!Controller1.ButtonX.pressing());
    }
    
    if(Button_Y) {
      // 设置到场地中心
      localization.setPosition(72.0, 72.0, 0.0);
      Brain.Screen.setCursor(5, 1);
      Brain.Screen.print("Position Set to Center");
      printf("已重定位到场地中心\n");
      waitUntil(!Controller1.ButtonY.pressing());
    }
    
    // 方向键用于微调位置（调试用）
    if(Button_Up) {
      double x, y, heading;
      localization.getPosition(x, y, heading);
      localization.setPosition(x, y + 6.0, heading); // 向前移动6英寸
      waitUntil(!Controller1.ButtonUp.pressing());
    }
    
    if(Button_Down) {
      double x, y, heading;
      localization.getPosition(x, y, heading);
      localization.setPosition(x, y - 6.0, heading); // 向后移动6英寸
      waitUntil(!Controller1.ButtonDown.pressing());
    }
    
    if(Button_Left) {
      double x, y, heading;
      localization.getPosition(x, y, heading);
      localization.setPosition(x - 6.0, y, heading); // 向左移动6英寸
      waitUntil(!Controller1.ButtonLeft.pressing());
    }
    
    if(Button_Right) {
      double x, y, heading;
      localization.getPosition(x, y, heading);
      localization.setPosition(x + 6.0, y, heading); // 向右移动6英寸
      waitUntil(!Controller1.ButtonRight.pressing());
    }

    wait(20, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() 
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(drivercontrol);

  // 运行预自动初始化
  pre_auton();

  // 显示欢迎信息
  printf("=== VEX机器人程序启动 ===\n");
  printf("编译时间: %s %s\n", __DATE__, __TIME__);
  
  // 检查设备状态
  while(1)
  {
    if(!imu.installed())
    {
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.setPenColor(red);
      Brain.Screen.print("错误: 未检测到IMU!");
      printf("错误: 未检测到IMU!\n");
    }
    else if (!localization_initialized)
    {
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.setPenColor(yellow);
      Brain.Screen.print("警告: 定位系统未初始化");
      printf("警告: 定位系统未初始化\n");
    }
    else
    {
      // 系统正常
      break;
    }
    
    wait(1000, msec);
  }

  // 主循环 - 保持程序运行
  while(true) {
    wait(1000, msec);
  }
  
  return 0;
}