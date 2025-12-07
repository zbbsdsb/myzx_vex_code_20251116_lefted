#include "vex.h"
#include "cmath"

void SHAIControl(int power)
{
  if (power == 0)
  {
    SHAI.stop(coast);
   

  }
  else
  {
    SHAI.spin(fwd, 0.128 * power,volt);
   
  }
}//滚筒Control
void DINGBUControl(int power)
{
  if (power == 0)
  {
    DINGBU.stop(coast);

  }
  else
  {
    DINGBU.spin(fwd, 0.128 * power,volt);
  }
}//滚筒Control





//========================================================

void Move(int left_power,int right_power)
{
  Left_Motor1.spin(forward, 0.128*left_power ,volt);
  Left_Motor2.spin(forward, 0.128*left_power ,volt);
  Left_Motor3.spin(forward, 0.128*left_power ,volt);

  Right_Motor1.spin(forward, 0.128*right_power ,volt);
  Right_Motor2.spin(forward, 0.128*right_power ,volt);
  Right_Motor3.spin(forward, 0.128*right_power ,volt);
}

void Move_Forward(int left_power,int right_power)
{
  Left_Motor1.spin(forward, left_power ,pct);
  Left_Motor2.spin(forward, left_power ,pct);
  Left_Motor3.spin(forward, left_power ,pct);

  Right_Motor1.spin(forward, right_power ,pct);
  Right_Motor2.spin(forward, right_power ,pct);
  Right_Motor3.spin(forward, right_power ,pct);
}

void Move_Reverse(int left_power,int right_power)
{
  Left_Motor1.spin(reverse, left_power ,pct);
  Left_Motor2.spin(reverse, left_power ,pct);
  Left_Motor3.spin(reverse, left_power ,pct);

  Right_Motor1.spin(reverse, right_power ,pct);
  Right_Motor2.spin(reverse, right_power ,pct);
  Right_Motor3.spin(reverse, right_power ,pct);
}

void Move_Stop_brake(void)
{
  Left_Motor1.stop(brake);
  Left_Motor2.stop(brake);
  Left_Motor3.stop(brake);
  Right_Motor1.stop(brake);
  Right_Motor2.stop(brake);
  Right_Motor3.stop(brake);
}

void Move_Stop_coast(void)
{
  Left_Motor1.stop(coast);
  Left_Motor2.stop(coast);
  Left_Motor3.stop(coast);
  Right_Motor1.stop(coast);
  Right_Motor2.stop(coast);
  Right_Motor3.stop(coast);
}

void Move_Stop_hold(void)
{
  Left_Motor1.stop(hold);
  Left_Motor2.stop(hold);
  Left_Motor3.stop(hold);
  Right_Motor1.stop(hold);
  Right_Motor2.stop(hold);
  Right_Motor3.stop(hold);
}

