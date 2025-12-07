#include "vex.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <vector>

#include "Move_Motor.h"


// ============================================================================
// 内部状态（请勿修改）
// ============================================================================
bool is_turning = false;
double prev_left_output = 0, prev_right_output = 0;
double x_pos = 0, y_pos = 0;
double correct_angle = 0;

// ============================================================================
// 底盘控制函数
motor left_chassis1 = motor(PORT1, ratio6_1, false);
motor left_chassis2 = motor(PORT2, ratio6_1, false);
motor left_chassis3 = motor(PORT3, ratio6_1, false);
motor_group left_chassis = motor_group(left_chassis1, left_chassis2, left_chassis3);
motor right_chassis1 = motor(PORT8, ratio6_1, true);
motor right_chassis2 = motor(PORT9, ratio6_1, true);
motor right_chassis3 = motor(PORT10, ratio6_1, true);
motor_group right_chassis = motor_group(right_chassis1, right_chassis2, right_chassis3);
// ============================================================================

/*
 * 设置左右底盘电机的电压输出
 * - left_power: 左侧电压（单位：伏特）
 * - right_power: 右侧电压（单位：伏特）
 */
void driveChassis(double left_power, double right_power) {
  // 以指定电压旋转左右底盘电机
  Left_Motor1.spin(fwd, left_power, voltageUnits::volt);
  Left_Motor2.spin(fwd, left_power, voltageUnits::volt);
  Left_Motor3.spin(fwd, left_power, voltageUnits::volt);
  Right_Motor1.spin(fwd, right_power, voltageUnits::volt);
  Right_Motor2.spin(fwd, right_power, voltageUnits::volt);
  Right_Motor3.spin(fwd, right_power, voltageUnits::volt);
}

/*
 * 使用指定制动类型停止两个底盘电机
 * - type: 制动模式（滑行、制动或保持）
 */
void stopChassis(brakeType type) {
  // 使用给定的制动类型停止左右底盘电机
  Left_Motor1.stop(type);
  Left_Motor2.stop(type);
  Left_Motor3.stop(type);
  Right_Motor1.stop(type);
  Right_Motor2.stop(type);
  Right_Motor3.stop(type);
}

/*
 * 将两个底盘电机的旋转位置重置为零
 */
void resetChassis() {
  // 将两个底盘电机编码器设置为零
  Left_Motor1.setPosition(0, degrees);
  Left_Motor2.setPosition(0, degrees);
  Left_Motor3.setPosition(0, degrees);
  Right_Motor1.setPosition(0, degrees);
  Right_Motor2.setPosition(0, degrees);
  Right_Motor3.setPosition(0, degrees);
}

/*
 * 返回左侧底盘电机当前的旋转角度（度）
 */
double getLeftRotationDegree() {
  // 获取左侧底盘电机位置（度）
  return left_chassis.position(degrees);
}

/*++++++++++++++++++++
 * 返回右侧底盘电机当前的旋转角度（度）
 */
double getRightRotationDegree() {
  // 获取右侧底盘电机位置（度）
  return right_chassis.position(degrees);
}

/*
 * 将角度标准化为当前航向的 +/-180 度范围内
 * - angle: 要标准化的目标角度
 */
double normalizeTarget(double angle) {
  // 调整角度使其在惯性传感器旋转的 +/-180 度范围内
  if (angle - inertial_sensor.rotation() > 180) {
    while (angle - inertial_sensor.rotation() > 180) angle -= 360;
  } else if (angle - inertial_sensor.rotation() < -180) {
    while (angle - inertial_sensor.rotation() < -180) angle += 360;
  }
  return angle;
}

/*
 * 返回当前惯性传感器航向（度）
 * - normalize: 如果为true，标准化航向（在此实现中未使用）
 */
double getInertialHeading(bool normalize) {
  // 获取惯性传感器旋转角度（度）
  return inertial_sensor.rotation(degrees);
}

// ============================================================================
// 输出缩放辅助函数
// ============================================================================

/*
 * 确保输出值至少达到两侧指定的最小值
 * - left_output: 左侧输出电压的引用
 * - right_output: 右侧输出电压的引用
 * - min_output: 允许的最小输出电压
 */
void scaleToMin(double& left_output, double& right_output, double min_output) {
  // 缩放输出以确保两侧都达到最小电压
  if (fabs(left_output) <= fabs(right_output) && left_output < min_output && left_output > 0) {
    right_output = right_output / left_output * min_output;
    left_output = min_output;
  } else if (fabs(right_output) < fabs(left_output) && right_output < min_output && right_output > 0) {
    left_output = left_output / right_output * min_output;
    right_output = min_output;
  } else if (fabs(left_output) <= fabs(right_output) && left_output > -min_output && left_output < 0) {
    right_output = right_output / left_output * -min_output;
    left_output = -min_output;
  } else if (fabs(right_output) < fabs(left_output) && right_output > -min_output && right_output < 0) {
    left_output = left_output / right_output * -min_output;
    right_output = -min_output;
  }
}

/*
 * 确保输出值不超过两侧指定的最大值
 * - left_output: 左侧输出电压的引用
 * - right_output: 右侧输出电压的引用
 * - max_output: 允许的最大输出电压
 */
void scaleToMax(double& left_output, double& right_output, double max_output) {
  // 缩放输出以确保两侧都不超过最大电压
  if (fabs(left_output) >= fabs(right_output) && left_output > max_output) {
    right_output = right_output / left_output * max_output;
    left_output = max_output;
  } else if (fabs(right_output) > fabs(left_output) && right_output > max_output) {
    left_output = left_output / right_output * max_output;
    right_output = max_output;
  } else if (fabs(left_output) > fabs(right_output) && left_output < -max_output) {
    right_output = right_output / left_output * -max_output;
    left_output = -max_output;
  } else if (fabs(right_output) > fabs(left_output) && right_output < -max_output) {
    left_output = left_output / right_output * -max_output;
    right_output = -max_output;
  }
}

// ============================================================================
// 主要驱动和转向函数
// ============================================================================

/*
 * 使用PID控制将机器人转向指定角度
 * - turn_angle: 目标转向角度（度）
 * - time_limit_msec: 转向允许的最大时间（毫秒）
 * - exit: 如果为true，在结束时停止机器人；如果为false，允许连锁动作
 * - max_output: 电机的最大电压输出
 */
void turnToAngle(double turn_angle, double time_limit_msec, bool exit, double max_output) {
  // 准备转向
  stopChassis(vex::brakeType::coast);
  is_turning = true;
  double threshold = 1;
  PID pid = PID(turn_kp, turn_ki, turn_kd);

  // 标准化并设置PID目标
  turn_angle = normalizeTarget(turn_angle);
  pid.setTarget(turn_angle);
  pid.setIntegralMax(0);  
  pid.setIntegralRange(3);
  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(50, 250);
  pid.setDerivativeTolerance(threshold * 4.5);

  // 绘制基线用于可视化
  double draw_amplifier = 230 / fabs(turn_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(turn_angle) * draw_amplifier, 600, fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);

  // 转向的PID循环
  double start_time = Brain.timer(msec);
  double output;
  double current_heading = getInertialHeading();
  double previous_heading = 0;
  int index = 1;
  if(exit == false && correct_angle < turn_angle) {
    // 向右转向，结束时不停车
    while (getInertialHeading() < turn_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading); // 航向的PID更新
      // 绘制航向轨迹
      Brain.Screen.drawLine(index * 3, fabs(previous_heading) * draw_amplifier, (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      // 钳制输出
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;
      driveChassis(output, -output);
      wait(10, msec);
    }
  } else if(exit == false && correct_angle > turn_angle) {
    // 向左转向，结束时不停车
    while (getInertialHeading() > turn_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);
      Brain.Screen.drawLine(index * 3, fabs(previous_heading) * draw_amplifier, (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;
      driveChassis(-output, output);
      wait(10, msec);
    }
  } else {
    // 标准PID转向
    while (!pid.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);
      Brain.Screen.drawLine(index * 3, fabs(previous_heading) * draw_amplifier, (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;
      driveChassis(output, -output);
      wait(10, msec);
    }
  }
  if(exit) {
    stopChassis(vex::hold);
  }
  correct_angle = turn_angle;
  is_turning = false;
}

/*
 * 使用PID控制驱动机器人指定距离（英寸）
 * - distance_in: 目标驱动距离（正或负）
 * - time_limit_msec: 移动允许的最大时间（毫秒）
 * - exit: 如果为true，在结束时停止机器人；如果为false，允许连锁动作
 * - max_output: 电机的最大电压输出
 */
void driveTo(double distance_in, double time_limit_msec, bool exit, double max_output) {
  // 存储初始编码器值
  double start_left = getLeftRotationDegree(), start_right = getRightRotationDegree();
  stopChassis(vex::brakeType::coast);
  is_turning = true;
  double threshold = 0.5;
  int drive_direction = distance_in > 0 ? 1 : -1;
  double max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    // 为连锁动作调整斜率率和最小速度
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = drive_direction > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  distance_in = distance_in * drive_direction;
  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  // 配置PID控制器
  pid_distance.setTarget(distance_in);
  pid_distance.setIntegralMax(3);  
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);

  pid_heading.setTarget(normalizeTarget(correct_angle));
  pid_heading.setIntegralMax(0);  
  pid_heading.setIntegralRange(1);
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0;
  double current_distance = 0, current_angle = 0;

  // 直线驱动的主要PID循环
  while (((!pid_distance.targetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec && exit) || (exit == false && current_distance < distance_in && Brain.timer(msec) - start_time <= time_limit_msec)) {
    // 计算当前距离和航向
    current_distance = (fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in) + fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in)) / 2;
    current_angle = getInertialHeading();
    left_output = pid_distance.update(current_distance) * drive_direction;
    right_output = left_output;
    correction_output = pid_heading.update(current_angle);

    // 最小输出检查
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }
    if(!exit) {
      left_output = 24 * drive_direction;
      right_output = 24 * drive_direction;
    }

    left_output += correction_output;
    right_output -= correction_output;

    // 最大输出检查
    scaleToMax(left_output, right_output, max_output);

    // 最大加速度/减速度检查
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output);
    wait(10, msec);
  }
  if(exit) {
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(vex::hold);
  }
  is_turning = false;
}

/*
 * 曲线圆
 * 以指定半径和角度驱动机器人沿圆弧运动
 * - result_angle_deg: 圆弧的目标结束角度（度）
 * - center_radius: 圆心的半径（正值为向右曲线，负值为向左曲线）
 * - time_limit_msec: 曲线运动允许的最大时间（毫秒）
 * - exit: 如果为true，在结束时停止机器人；如果为false，允许连锁动作
 * - max_output: 电机的最大电压输出
 */
void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit, double max_output) {
  // 存储两侧的初始编码器值
  double start_right = getRightRotationDegree(), start_left = getLeftRotationDegree();
  double in_arc, out_arc;
  double real_angle = 0, current_angle = 0;
  double ratio, result_angle;

  // 将目标角度标准化为当前航向的 +/-180 度范围内
  result_angle_deg = normalizeTarget(result_angle_deg);
  result_angle = (result_angle_deg - correct_angle) * 3.14159265359 / 180;

  // 计算内外轮的弧长
  in_arc = fabs((fabs(center_radius) - (distance_between_wheels / 2)) * result_angle);
  out_arc = fabs((fabs(center_radius) + (distance_between_wheels / 2)) * result_angle);
  ratio = in_arc / out_arc;

  stopChassis(vex::brakeType::coast);
  is_turning = true;
  double threshold = 0.5;

  // 确定曲线和驱动方向
  int curve_direction = center_radius > 0 ? 1 : -1;
  int drive_direction = 0;
  if ((curve_direction == 1 && (result_angle_deg - correct_angle) > 0) || (curve_direction == -1 && (result_angle_deg - correct_angle) < 0)) {
    drive_direction = 1;
  } else {
    drive_direction = -1;
  }

  // 连锁动作的斜率率和最小速度逻辑
  double max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = drive_direction > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  // 初始化弧长距离和航向校正的PID控制器
  PID pid_out = PID(distance_kp, distance_ki, distance_kd);
  PID pid_turn = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  pid_out.setTarget(out_arc);
  pid_out.setIntegralMax(0);  
  pid_out.setIntegralRange(5);
  pid_out.setSmallBigErrorTolerance(0.3, 0.9);
  pid_out.setSmallBigErrorDuration(50, 250);
  pid_out.setDerivativeTolerance(threshold * 4.5);

  pid_turn.setTarget(0);
  pid_turn.setIntegralMax(0);  
  pid_turn.setIntegralRange(1);
  pid_turn.setSmallBigErrorTolerance(0, 0);
  pid_turn.setSmallBigErrorDuration(0, 0);
  pid_turn.setDerivativeTolerance(0);
  pid_turn.setArrive(false);

  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0;
  double current_right = 0, current_left = 0;

  // 每个曲线/退出配置的主要控制循环
  if (curve_direction == -1 && exit == true) {
    // 左曲线，结束时停止
    while (!pid_out.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_right = fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in);
      // 计算沿弧的实际角度
      real_angle = current_right/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      right_output = pid_out.update(current_right) * drive_direction;
      left_output = right_output * ratio;
      correction_output = pid_turn.update(current_angle);

      // 如果连锁，强制执行最小输出
      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      // 应用航向校正
      left_output += correction_output;
      right_output -= correction_output;

      // 强制执行最大输出
      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  } else if (curve_direction == 1 && exit == true) {
    // 右曲线，结束时停止
    while (!pid_out.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_left = fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in);
      real_angle = current_left/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      left_output = pid_out.update(current_left) * drive_direction;
      right_output = left_output * ratio;
      correction_output = pid_turn.update(current_angle);

      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      left_output += correction_output;
      right_output -= correction_output;

      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  } else if (curve_direction == -1 && exit == false) {
    // 左曲线，连锁（结束时不停车）
    while (current_right < out_arc && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_right = fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in);
      real_angle = current_right/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      right_output = pid_out.update(current_right) * drive_direction;
      left_output = right_output * ratio;
      correction_output = pid_turn.update(current_angle);

      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }
      
      left_output += correction_output;
      right_output -= correction_output;

      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  } else {
    // 右曲线，连锁（结束时不停车）
    while (current_left < out_arc && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_left = fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in);
      real_angle = current_left/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      left_output = pid_out.update(current_left) * drive_direction;
      right_output = left_output * ratio;
      correction_output = pid_turn.update(current_angle);

      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      left_output += correction_output;
      right_output -= correction_output;

      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  }
  // 如果需要，停止底盘
  if(exit == true) {
    stopChassis(vex::brakeType::hold);
  }
  // 更新全局航向
  correct_angle = result_angle_deg;
  is_turning = false;
}

/*
 * 摆动转向
 * 执行摆动转向，在向前或向后驱动时绕一个点旋转机器人
 * - swing_angle: 目标摆动角度（度）
 * - drive_direction: 驱动方向（1为向前，-1为向后）
 * - time_limit_msec: 摆动允许的最大时间（毫秒）
 * - exit: 如果为true，在结束时停止机器人；如果为false，允许连锁动作
 * - max_output: 电机的最大电压输出
 */
void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit, double max_output) {
  stopChassis(vex::brakeType::coast); // 开始摆动前停止底盘
  is_turning = true;                  // 设置转向状态
  double threshold = 1;
  PID pid = PID(turn_kp, turn_ki, turn_kd); // 初始化转向PID

  swing_angle = normalizeTarget(swing_angle); // 标准化目标角度
  pid.setTarget(swing_angle);                 // 设置PID目标
  pid.setIntegralMax(0);  
  pid.setIntegralRange(5);

  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(50, 250);
  pid.setDerivativeTolerance(threshold * 4.5);

  // 绘制基线用于可视化
  double draw_amplifier = 230 / fabs(swing_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(swing_angle) * draw_amplifier, 600, fabs(swing_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);

  // 开始PID循环
  double start_time = Brain.timer(msec);
  double output;
  double current_heading = correct_angle;
  double previous_heading = 0;
  int index = 1;
  int choice = 1;

  // 确定摆动方向和驱动方向
  if(swing_angle - correct_angle < 0 && drive_direction == 1) {
    choice = 1;
  } else if(swing_angle - correct_angle > 0 && drive_direction == 1) {
    choice = 2;
  } else if(swing_angle - correct_angle < 0 && drive_direction == -1) {
    choice = 3;
  } else {
    choice = 4;
  }

  // 每种情况的摆动逻辑，连锁（exit == false）
  if(choice == 1 && exit == false) {
    // 左摆动，向前
    while (current_heading > swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // 绘制航向轨迹
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // 钳制输出
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.stop(hold); // 保持左侧，摆动右侧
      right_chassis.spin(fwd, output * drive_direction, volt);
      wait(10, msec);
    }
  } else if(choice == 2 && exit == false) {
    // 右摆动，向前
    while (current_heading < swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // 绘制航向轨迹
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // 钳制输出
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.spin(fwd, output * drive_direction, volt);
      right_chassis.stop(hold); // 保持右侧，摆动左侧
      wait(10, msec);
    }
  } else if(choice == 3 && exit == false) {
    // 左摆动，向后
    while (current_heading > swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // 绘制航向轨迹
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // 钳制输出
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.spin(fwd, output * drive_direction, volt);
      right_chassis.stop(hold);
      wait(10, msec);
    }
  } else {
    // 右摆动，向后
    while (current_heading < swing_angle && Brain.timer(msec) - start_time <= time_limit_msec && exit == false) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // 绘制航向轨迹
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // 钳制输出
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.stop(hold);
      right_chassis.spin(fwd, output * drive_direction, volt);
      wait(10, msec);
    }
  }

  // exit == true 的PID循环（结束时停止）
  while (!pid.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec && exit == true) {
    current_heading = getInertialHeading();
    output = pid.update(current_heading);

    // 绘制航向轨迹
    Brain.Screen.drawLine(
        index * 3, fabs(previous_heading) * draw_amplifier, 
        (index + 1) * 3, fabs(current_heading * draw_amplifier));
    index++;
    previous_heading = current_heading;

    // 钳制输出
    if(output > max_output) output = max_output;
    else if(output < -max_output) output = -max_output;

    // 根据摆动方向将输出应用到正确的一侧
    switch(choice) {
    case 1:
      left_chassis.stop(hold);
      right_chassis.spin(fwd, -output * drive_direction, volt);
      break;
    case 2:
      left_chassis.spin(fwd, output * drive_direction, volt);
      right_chassis.stop(hold);
      break;
    case 3:
      left_chassis.spin(fwd, -output * drive_direction, volt);
      right_chassis.stop(hold);
      break;
    case 4:
      left_chassis.stop(hold);
      right_chassis.spin(fwd, output * drive_direction, volt);
      break;
    }
    wait(10, msec);
  }
  if(exit == true) {
    stopChassis(vex::hold); // 如果需要，在结束时停止底盘
  }
  correct_angle = swing_angle; // 更新全局航向
  is_turning = false;          // 重置转向状态
}

/*
 * 航向校正
 * 持续调整机器人的航向以保持直线行驶
 * 使用PID控制器来最小化当前航向与目标航向之间的误差
 */
void correctHeading() {
  double output = 0;
  PID pid = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  pid.setTarget(correct_angle); // 设置PID目标为当前航向
  pid.setIntegralRange(fabs(correct_angle) / 2.5);

  pid.setSmallBigErrorTolerance(0, 0);
  pid.setSmallBigErrorDuration(0, 0);
  pid.setDerivativeTolerance(0);
  pid.setArrive(false);

  // 在启用时持续校正航向
  while(heading_correction) {
    pid.setTarget(correct_angle);
    if(is_turning == false) {
      output = pid.update(getInertialHeading());
      driveChassis(output, -output); // 对底盘应用校正
    }
    wait(10, msec);
  }
}

/*
 * 无里程轮追踪
 * 仅使用驱动编码器和惯性传感器追踪机器人位置
 * 假设没有外部里程追踪轮
 */
void trackNoOdomWheel() {
  resetChassis();
  double prev_heading_rad = 0;
  double prev_left_deg = 0, prev_right_deg = 0;
  double delta_local_y_in = 0;

  while (true) {
    double heading_rad = degToRad(getInertialHeading());
    double left_deg = getLeftRotationDegree();
    double right_deg = getRightRotationDegree();
    double delta_heading_rad = heading_rad - prev_heading_rad; // 航向变化（弧度）
    double delta_left_in = (left_deg - prev_left_deg) * wheel_distance_in / 360.0;   // 左轮增量（英寸）
    double delta_right_in = (right_deg - prev_right_deg) * wheel_distance_in / 360.0; // 右轮增量（英寸）
    // 如果没有航向变化，视为直线运动
    if (fabs(delta_heading_rad) < 1e-6) {
      delta_local_y_in = (delta_left_in + delta_right_in) / 2.0;
    } else {
      // 计算每个轮的弧线运动
      double sin_multiplier = 2.0 * sin(delta_heading_rad / 2.0);
      double delta_local_y_left_in = sin_multiplier * (delta_left_in / delta_heading_rad + distance_between_wheels / 2.0);
      double delta_local_y_right_in = sin_multiplier * (delta_right_in / delta_heading_rad + distance_between_wheels / 2.0);
      delta_local_y_in = (delta_local_y_left_in + delta_local_y_right_in) / 2.0;
    }
    // 使用极坐标更新全局位置
    double polar_angle_rad = prev_heading_rad + delta_heading_rad / 2.0;
    double polar_radius_in = delta_local_y_in;

    x_pos += polar_radius_in * sin(polar_angle_rad);
    y_pos += polar_radius_in * cos(polar_angle_rad);

    prev_heading_rad = heading_rad;
    prev_left_deg = left_deg;
    prev_right_deg = right_deg;

    wait(10, msec);
  }
}

/*
 * XY里程轮追踪
 * 使用水平和垂直里程轮加惯性航向追踪机器人位置
 */
void trackXYOdomWheel() {
  resetChassis();
  double prev_heading_rad = 0;
  double prev_horizontal_pos_deg = 0, prev_vertical_pos_deg = 0;
  double delta_local_x_in = 0, delta_local_y_in = 0;
  double local_polar_angle_rad = 0;

  while (true) {
    double heading_rad = degToRad(getInertialHeading());
    double horizontal_pos_deg = horizontal_tracker.position(degrees);
    double vertical_pos_deg = vertical_tracker.position(degrees);
    double delta_heading_rad = heading_rad - prev_heading_rad;
    double delta_horizontal_in = (horizontal_pos_deg - prev_horizontal_pos_deg) * horizontal_tracker_diameter * M_PI / 360.0; // 水平追踪器增量（英寸）
    double delta_vertical_in = (vertical_pos_deg - prev_vertical_pos_deg) * vertical_tracker_diameter * M_PI / 360.0; // 垂直追踪器增量（英寸）

    // 基于航向变化计算局部运动
    if (fabs(delta_heading_rad) < 1e-6) {
      delta_local_x_in = delta_horizontal_in;
      delta_local_y_in = delta_vertical_in;
    } else {
      double sin_multiplier = 2.0 * sin(delta_heading_rad / 2.0);
      delta_local_x_in = sin_multiplier * ((delta_horizontal_in / delta_heading_rad) + horizontal_tracker_dist_from_center);
      delta_local_y_in = sin_multiplier * ((delta_vertical_in / delta_heading_rad) + vertical_tracker_dist_from_center);
    }

    // 避免 atan2(0, 0) 未定义
    if (fabs(delta_local_x_in) < 1e-6 && fabs(delta_local_y_in) < 1e-6) {
      local_polar_angle_rad = 0;
    } else {
      local_polar_angle_rad = atan2(delta_local_y_in, delta_local_x_in);
    }
    double polar_radius_in = sqrt(pow(delta_local_x_in, 2) + pow(delta_local_y_in, 2));
    double polar_angle_rad = local_polar_angle_rad - heading_rad - (delta_heading_rad / 2);

    x_pos += polar_radius_in * cos(polar_angle_rad);
    y_pos += polar_radius_in * sin(polar_angle_rad);

    prev_heading_rad = heading_rad;
    prev_horizontal_pos_deg = horizontal_pos_deg;
    prev_vertical_pos_deg = vertical_pos_deg;

    wait(10, msec);
  }
}

/*
 * X里程轮追踪
 * 仅使用水平里程轮 + 驱动编码器 + 惯性航向追踪位置
 */
void trackXOdomWheel() {
  resetChassis();
  double prev_heading_rad = 0;
  double prev_horizontal_pos_deg = 0;
  double prev_left_deg = 0, prev_right_deg = 0;
  double delta_local_x_in = 0, delta_local_y_in = 0;
  double local_polar_angle_rad = 0;

  while (true) {
    double heading_rad = degToRad(getInertialHeading());
    double horizontal_pos_deg = horizontal_tracker.position(degrees);
    double left_deg = getLeftRotationDegree();
    double right_deg = getRightRotationDegree();
    double delta_heading_rad = heading_rad - prev_heading_rad;
    double delta_horizontal_in = (horizontal_pos_deg - prev_horizontal_pos_deg) * horizontal_tracker_diameter * M_PI / 360.0; // 水平追踪器增量（英寸）
    double delta_left_in = (left_deg - prev_left_deg) * wheel_distance_in / 360.0;   // 左轮增量（英寸）
    double delta_right_in = (right_deg - prev_right_deg) * wheel_distance_in / 360.0; // 右轮增量（英寸）

    // 基于航向变化计算局部运动
    if (fabs(delta_heading_rad) < 1e-6) {
      delta_local_x_in = delta_horizontal_in;
      delta_local_y_in = (delta_left_in + delta_right_in) / 2.0;
    } else {
      double sin_multiplier = 2.0 * sin(delta_heading_rad / 2.0);
      delta_local_x_in = sin_multiplier * ((delta_horizontal_in / delta_heading_rad) + horizontal_tracker_dist_from_center);
      double delta_local_y_left_in = sin_multiplier * (delta_left_in / delta_heading_rad + distance_between_wheels / 2.0);
      double delta_local_y_right_in = sin_multiplier * (delta_right_in / delta_heading_rad + distance_between_wheels / 2.0);
      delta_local_y_in = (delta_local_y_left_in + delta_local_y_right_in) / 2.0;
    }

    if (fabs(delta_local_x_in) < 1e-6 && fabs(delta_local_y_in) < 1e-6) {
      local_polar_angle_rad = 0;
    } else {
      local_polar_angle_rad = atan2(delta_local_y_in, delta_local_x_in);
    }
    double polar_radius_in = sqrt(pow(delta_local_x_in, 2) + pow(delta_local_y_in, 2));
    double polar_angle_rad = local_polar_angle_rad - heading_rad - (delta_heading_rad / 2);

    x_pos += polar_radius_in * cos(polar_angle_rad);
    y_pos += polar_radius_in * sin(polar_angle_rad);
    
    prev_heading_rad = heading_rad;
    prev_horizontal_pos_deg = horizontal_pos_deg;
    prev_left_deg = left_deg;
    prev_right_deg = right_deg;

    wait(10, msec);
  }
}

/*
 * Y里程轮追踪
 * 仅使用垂直里程轮 + 惯性航向追踪位置
 */
void trackYOdomWheel() {
  resetChassis();
  double prev_heading_rad = 0;
  double prev_vertical_pos_deg = 0;
  double delta_local_y_in = 0;

  while (true) {
    double heading_rad = degToRad(getInertialHeading());
    double vertical_pos_deg = vertical_tracker.position(degrees);
    double delta_heading_rad = heading_rad - prev_heading_rad;
    double delta_vertical_in = (vertical_pos_deg - prev_vertical_pos_deg) * vertical_tracker_diameter * M_PI / 360.0; // 垂直追踪器增量（英寸）

    // 基于航向变化计算局部运动
    if (fabs(delta_heading_rad) < 1e-6) {
      delta_local_y_in = delta_vertical_in;
    } else {
      double sin_multiplier = 2.0 * sin(delta_heading_rad / 2.0);
      delta_local_y_in = sin_multiplier * ((delta_vertical_in / delta_heading_rad) + vertical_tracker_dist_from_center);
    }

    double polar_angle_rad = prev_heading_rad + delta_heading_rad / 2.0;
    double polar_radius_in = delta_local_y_in;

    x_pos += polar_radius_in * cos(polar_angle_rad);
    y_pos += polar_radius_in * sin(polar_angle_rad);

    prev_heading_rad = heading_rad;
    prev_vertical_pos_deg = vertical_pos_deg;

    wait(10, msec);
  }
}

/*
 * 转向点
 * 将机器人转向面对场地上特定点
 * - x, y: 目标点的坐标
 * - direction: 面对点的方向（1为向前，-1为向后）
 * - time_limit_msec: 转向允许的最大时间（毫秒）
 */
void turnToPoint(double x, double y, int direction, double time_limit_msec) {
  stopChassis(vex::brakeType::coast); // 转向前停止底盘
  is_turning = true;                  // 设置转向状态
  double threshold = 1, add = 0;
  if(direction == -1) {
    add = 180; // 如果要向后面对，增加180度
  }
  // 使用 atan2 计算目标角度并标准化
  double turn_angle = normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos))) + add;
  PID pid = PID(turn_kp, turn_ki, turn_kd);

  pid.setTarget(turn_angle); // 设置PID目标
  pid.setIntegralMax(0);  
  pid.setIntegralRange(3);

  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(100, 500);
  pid.setDerivativeTolerance(threshold * 4.5);

  // 绘制基线用于可视化
  double draw_amplifier = 230 / fabs(turn_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(turn_angle) * draw_amplifier, 
                        600, fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);

  // 开始PID循环
  double start_time = Brain.timer(msec);
  double output;
  double current_heading;
  double previous_heading = 0;
  int index = 1;
  while (!pid.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
    // 随着机器人移动持续更新目标
    pid.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos))) + add);
    current_heading = getInertialHeading();
    output = pid.update(current_heading);

    // 绘制航向轨迹
    Brain.Screen.drawLine(
        index * 3, fabs(previous_heading) * draw_amplifier, 
        (index + 1) * 3, fabs(current_heading * draw_amplifier));
    index++;
    previous_heading = current_heading;

    driveChassis(output, -output); // 对底盘应用输出
    wait(10, msec);
  }  
  stopChassis(vex::hold); // 结束时停止
  correct_angle = getInertialHeading(); // 更新全局航向
  is_turning = false;                   // 重置转向状态
}

/*
 * 移动到点
 * 将机器人移动到场地上特定点，根据需要调整航向
 * - x, y: 目标点的坐标
 * - dir: 移动方向（1为向前，-1为向后）
 * - time_limit_msec: 移动允许的最大时间（毫秒）
 * - exit: 如果为true，在结束时停止机器人；如果为false，允许连锁动作
 * - max_output: 电机的最大电压输出
 * - overturn: 如果为true，允许急转弯
 */
void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit, double max_output, bool overturn) {
  stopChassis(vex::brakeType::coast); // 移动前停止底盘
  is_turning = true;                  // 设置转向状态
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double max_slew_fwd = dir > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = dir > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    // 为连锁动作调整斜率率和最小速度
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = dir > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = dir > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = dir > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = dir > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  // 设置距离和航向的PID目标
  pid_distance.setTarget(hypot(x - x_pos, y - y_pos));
  pid_distance.setIntegralMax(0);  
  pid_distance.setIntegralRange(3);
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);
  
  pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add));
  pid_heading.setIntegralMax(0);  
  pid_heading.setIntegralRange(1);
  
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  // 重置底盘
  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0, prev_left_output = 0, prev_right_output = 0;
  double exittolerance = 1;
  bool perpendicular_line = false, prev_perpendicular_line = true;

  double current_angle = 0, overturn_value = 0;
  bool ch = true;

  // 移动到点的主要PID循环
  while (Brain.timer(msec) - start_time <= time_limit_msec) {
    // 随着机器人移动持续更新目标
    pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add));
    pid_distance.setTarget(hypot(x - x_pos, y - y_pos));
    current_angle = getInertialHeading();
    // 基于航向和距离计算驱动输出
    left_output = pid_distance.update(0) * cos(degToRad(atan2(x - x_pos, y - y_pos) * 180 / M_PI + add - current_angle)) * dir;
    right_output = left_output;
    // 检查机器人是否已穿过目标的垂直线
    perpendicular_line = ((y_pos - y) * -cos(degToRad(normalizeTarget(current_angle + add))) <= (x_pos - x) * sin(degToRad(normalizeTarget(current_angle + add))) + exittolerance);
    if(perpendicular_line && !prev_perpendicular_line) {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    // 仅在远离目标时应用航向校正
    if(hypot(x - x_pos, y - y_pos) > 8 && ch == true) {
      correction_output = pid_heading.update(current_angle);
    } else {
      correction_output = 0;
      ch = false;
    }

    // 最小输出检查
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }

    // 急转弯逻辑
    overturn_value = fabs(left_output) + fabs(correction_output) - max_output;
    if(overturn_value > 0 && overturn) {
      if(left_output > 0) {
        left_output -= overturn_value;
      }
      else {
        left_output += overturn_value;
      }
    }
    right_output = left_output;
    left_output = left_output + correction_output;
    right_output = right_output - correction_output;

    // 最大输出检查
    scaleToMax(left_output, right_output, max_output);

    // 最大加速度/减速度检查
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output); // 对底盘应用输出
    wait(10, msec);
  }
  if(exit == true) {
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(vex::hold); // 如果需要，在结束时停止
  }
  correct_angle = getInertialHeading(); // 更新全局航向
  is_turning = false;                   // 重置转向状态
}

/*
 * 回旋镖
 * 以回旋镖形状的路径将机器人驱动到目标点
 * - x, y: 目标点的坐标
 * - a: 机器人到目标的最终角度（度）
 * - dlead: 领先目标的距离（英寸，设置较高值以获得更弯曲的路径，不要设置超过0.6）
 * - time_limit_msec: 机动允许的最大时间（毫秒）
 * - dir: 移动方向（1为向前，-1为向后）
 * - exit: 如果为true，在结束时停止机器人；如果为false，允许连锁动作
 * - max_output: 电机的最大电压输出
 * - overturn: 如果为true，允许急转弯
 */
void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit, double max_output, bool overturn) {
  stopChassis(vex::brakeType::coast); // 移动前停止底盘
  is_turning = true;                  // 设置转向状态
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double max_slew_fwd = dir > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = dir > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    // 为连锁动作调整斜率率和最小速度
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = dir > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = dir > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = dir > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = dir > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  pid_distance.setTarget(0); // 目标动态更新
  pid_distance.setIntegralMax(3);  
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);

  pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos))));
  pid_heading.setIntegralMax(0);  
  pid_heading.setIntegralRange(1);
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0, slip_speed = 0, overturn_value = 0;
  double exit_tolerance = 3;
  bool perpendicular_line = false, prev_perpendicular_line = true;
  double current_angle = 0, hypotenuse = 0, carrot_x = 0, carrot_y = 0;

  // 回旋镖路径的主要PID循环
  while ((!pid_distance.targetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec) {
    hypotenuse = hypot(x_pos - x, y_pos - y); // 到目标的距离
    // 计算路径引导的胡萝卜点
    carrot_x = x - hypotenuse * sin(degToRad(a + add)) * dlead;
    carrot_y = y - hypotenuse * cos(degToRad(a + add)) * dlead;
    pid_distance.setTarget(hypot(carrot_x - x_pos, carrot_y - y_pos) * dir);
    current_angle = getInertialHeading();
    // 基于胡萝卜点计算驱动输出
    left_output = pid_distance.update(0) * cos(degToRad(atan2(carrot_x - x_pos, carrot_y - y_pos) * 180 / M_PI + add - current_angle));
    right_output = left_output;
    // 检查机器人是否已穿过目标的垂直线
    perpendicular_line = ((y_pos - y) * -cos(degToRad(normalizeTarget(a))) <= (x_pos - x) * sin(degToRad(normalizeTarget(a))) + exit_tolerance);
    if(perpendicular_line && !prev_perpendicular_line) {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    // 最小输出检查
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }

    // 基于到胡萝卜点/目标的距离的航向校正逻辑
    if(hypot(carrot_x - x_pos, carrot_y - y_pos) > 8) {
      pid_heading.setTarget(normalizeTarget(radToDeg(atan2(carrot_x - x_pos, carrot_y - y_pos)) + add));
      correction_output = pid_heading.update(current_angle);
    } else if(hypot(x - x_pos, y - y_pos) > 6) {
      pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add));
      correction_output = pid_heading.update(current_angle);
    } else {
      pid_heading.setTarget(normalizeTarget(a));
      correction_output = pid_heading.update(current_angle);
      if(exit && hypot(x - x_pos, y - y_pos) < 5) {
        break;
      }
    }

    // 限制滑动速度以获得更平滑的曲线
    slip_speed = sqrt(chase_power * getRadius(x_pos, y_pos, carrot_x, carrot_y, current_angle) * 9.8);
    if(left_output > slip_speed) {
      left_output = slip_speed;
    } else if(left_output < -slip_speed) {
      left_output = -slip_speed;
    }

    // 急转弯逻辑
    overturn_value = fabs(left_output) + fabs(correction_output) - max_output;
    if(overturn_value > 0 && overturn) {
      if(left_output > 0) {
        left_output -= overturn_value;
      }
      else {
        left_output += overturn_value;
      }
    }
    right_output = left_output;
    left_output = left_output + correction_output;
    right_output = right_output - correction_output;

    // 最大输出检查
    scaleToMax(left_output, right_output, max_output);

    // 最大加速度/减速度检查
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output); // 对底盘应用输出
    wait(10, msec);
  }
  if(exit) {
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(vex::hold); // 如果需要，在结束时停止
  }
  correct_angle = a;      // 更新全局航向
  is_turning = false;     // 重置转向状态
}

// ============================================================================
// 模板说明
// ============================================================================
// 此文件旨在作为VEX/V5机器人团队的模板
// 所有函数和变量使用清晰、一致的命名约定
// 注释简明扼要，解释每个部分的意图
// 团队可以根据其机器人的需要调整PID值、驱动基座几何形状和逻辑