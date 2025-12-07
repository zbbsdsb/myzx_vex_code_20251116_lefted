#include "Move_Motor.h"
#include "PID.h"
#include "vex.h"

void RunAuto() {
imu.setRotation(0,deg);
B.set(0);
PID_Move(60,60,400,800);
PID_Turn(19);
PID_Move(60,60,250,800);
SHAIControl(-100);
PID_Move(25,25,350,800);
SHAIControl(0);
PID_Turn(-54);
PID_Move(25,25,330,800);
SHAIControl(70);
wait(700,msec);
SHAIControl(-100);
wait(400,msec);
SHAIControl(0);
PID_Move(60,60,-100,800);
PID_Turn(145);
PID_Move(60,60,1050,800);
wait(100,msec);
B.set(1);
PID_Turn(180);
SHAIControl(-100);
wait(300,msec);
PID_Move(100,100,350,800);
wait(750,msec);
PID_Move(60,60,-100,800);
PID_Turn(188);
PID_Move(60,60,-550,600);
DINGBUControl(-100);
}