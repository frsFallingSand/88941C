#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor L1 = motor(PORT11, ratio6_1, true);
motor L2 = motor(PORT12, ratio6_1, true);
motor L3 = motor(PORT13, ratio6_1, true);
motor R1 = motor(PORT18, ratio6_1, false);
motor R2 = motor(PORT19, ratio6_1, false);
motor R3 = motor(PORT20, ratio6_1, false);
motor Intake = motor(PORT10, ratio6_1, true);
motor Export = motor(PORT2, ratio18_1, false);
motor transmit = motor(PORT1, ratio18_1, true);

// 气缸
digital_out Long_bridge_baffle = digital_out(Brain.ThreeWirePort.F);
digital_out Import_bucket_baffle = digital_out(Brain.ThreeWirePort.G);
digital_out IntakeCylinder = digital_out(Brain.ThreeWirePort.B);
digital_out hook = digital_out(Brain.ThreeWirePort.D);
digital_out odometer = digital_out(Brain.ThreeWirePort.A);

// 传感器
inertial IMU = inertial(PORT8);
optical Optical = optical(PORT4);
rotation x = rotation(PORT7, true);
rotation y = rotation(PORT5, false);
// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
    // nothing to initialize
}
