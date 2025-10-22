#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor L1 = motor(PORT12, ratio6_1, true);
motor L2 = motor(PORT13, ratio6_1, false);
motor L3 = motor(PORT14, ratio6_1, true);
motor R1 = motor(PORT18, ratio6_1, false);
motor R2 = motor(PORT19, ratio6_1, false);
motor R3 = motor(PORT20, ratio6_1, true);
motor Intake = motor(PORT2, ratio18_1, false);
motor Export = motor(PORT3, ratio18_1, false);
inertial IMU = inertial(PORT10);
motor Intake2 = motor(PORT1, ratio18_1, true);
digital_out front_panel = digital_out(Brain.ThreeWirePort.A);
digital_out Double_hook = digital_out(Brain.ThreeWirePort.B);
digital_out Lift_it_up = digital_out(Brain.ThreeWirePort.H);
rotation x = rotation(PORT8, true);
rotation y = rotation(PORT15, true);

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