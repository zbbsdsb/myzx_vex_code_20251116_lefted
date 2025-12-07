#include "vex.h"
using namespace vex;

void handle_grab();

extern controller Controller1;
extern controller Controller2;
// extern inertial Inertial;
extern brain Brain;
const double DPMAXSPEED = 200*360/60;//电机最大速度dps

inline void check()
{
   int error = 0;
  // Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(3,0);
  printf("\n\ncheck.....");
  if(!Left_Motor1.installed())
  {
    error++;
    // Controller1.Screen.print("L1 ");
    Controller1.Screen.print("Left_Motor1 not installed\n");
  }
  if(!Left_Motor2.installed())
  {
    error++;
    // Controller1.Screen.print("L2");
    Controller1.Screen.print("Left_Motor2 not installed\n");
  }
    if(!Left_Motor3.installed())
  {
    error++;
    // Controller1.Screen.print("L2");
    Controller1.Screen.print("Left_Motor3 not installed\n");
  }
  if(!Right_Motor1.installed())
  {
    error++;
    // Controller1.Screen.print("R1");
    Controller1.Screen.print("Right_Motor1 not installed\n ");
  }
  if(!Right_Motor2.installed())
  {
    error++;
    // Controller1.Screen.print("R2");
    Controller1.Screen.print("Right_Motor2 not installed\n");
  }
    if(!Right_Motor3.installed())
  {
    error++;
    // Controller1.Screen.print("R2");
    Controller1.Screen.print("Right_Motor3 not installed\n");
  }
  if(!Right_Motor3.installed())
  {
    error++;
    // Controller1.Screen.print("INA ");
    Controller1.Screen.print("Catapult_Motor not installed\n ");
  }
  if(!Left_Motor3.installed())
  {
    error++;
    // Controller1.Screen.print("INB ");
    Controller1.Screen.print("Filling_Motor not installed\n ");
  }
  if(!SHAI.installed()){
    error++;
    // Controller1.Screen.print("Tr ");
    Controller1.Screen.print("SHAI_Motor not installed\n ");
  }
  
  


  if(error)
  {
    Controller1.Screen.print(" %d  ",error);
    Controller1.rumble("..-....-........-......--........-...");
    wait(100,msec);
    Controller1.rumble("..-....-.........");
  }
} 

inline void init()
{
    check();
    if(!imu.installed())
    {
        Controller1.rumble("...");
        printf("Inertial not installed!\n");
        Controller1.Screen.print("No Inertial!");
    }
    else
    {
      thread([]{
          imu.resetHeading();
          wait(100,msec);
          Controller1.rumble(".");
      });
    }
    // if(!Brain.SDcard.isInserted()) printf("SDcard not inserted!\n");
}
