using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor Left_Motor1;
extern motor Left_Motor2;
extern motor Left_Motor3;
extern motor Right_Motor1;
extern motor Right_Motor2;
extern motor Right_Motor3;
extern motor SHAI;
extern motor DINGBU;
extern inertial imu;
extern digital_out A;
extern digital_out B;
extern rotation Rotation19;
extern rotation Rotation20;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );