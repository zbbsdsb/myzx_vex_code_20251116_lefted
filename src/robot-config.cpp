#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor Left_Motor1 = motor(PORT1, ratio18_1, false);
motor Left_Motor2 = motor(PORT2, ratio18_1, false);
motor Left_Motor3 = motor(PORT3, ratio18_1, false);
motor Right_Motor1 = motor(PORT10, ratio18_1, true);
motor Right_Motor2 = motor(PORT9, ratio6_1, true);
motor Right_Motor3 = motor(PORT8, ratio18_1, true);
motor SHAI = motor(PORT7, ratio18_1, true);
motor DINGBU = motor(PORT11, ratio18_1, false);
inertial imu = inertial(PORT12);
digital_out A = digital_out(Brain.ThreeWirePort.A);
digital_out B = digital_out(Brain.ThreeWirePort.B);
rotation Rotation19 = rotation(PORT19, false);
rotation Rotation20 = rotation(PORT20, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}