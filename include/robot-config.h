using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor L1;
extern motor L2;
extern motor L3;
extern motor R1;
extern motor R2;
extern motor R3;
extern motor Intake;
extern motor Export;
extern inertial IMU;
extern motor Intake2;
extern digital_out front_panel;
extern digital_out Double_hook;
extern digital_out Lift_it_up;
extern rotation x;
extern rotation y;
extern optical Optical;
extern motor transmit;
extern digital_out IntakeCylinder;
extern digital_out Long_bridge_baffle;
extern digital_out Import_bucket_baffle;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
