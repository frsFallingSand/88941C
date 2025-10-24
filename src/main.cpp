// /*----------------------------------------------------------------------------*/
// /*                                                                            */
// /*    Module:       main.cpp                                                  */
// /*    Author:       fgsd                                                      */
// /*    Created:      2025/10/20 20:44:46                                       */
// /*    Description:  V5 project                                                */
// /*                                                                            */
// /*----------------------------------------------------------------------------*/

// #include "vex.h"

// using namespace vex;

// // A global instance of competition
// competition Competition;

// // define your global instances of motors and other devices here

// /*---------------------------------------------------------------------------*/
// /*                          Pre-Autonomous Functions                         */
// /*                                                                           */
// /*  You may want to perform some actions before the competition starts.      */
// /*  Do them in the following function.  You must return from this function   */
// /*  or the autonomous and usercontrol tasks will not be started.  This       */
// /*  function is only called once after the V5 has been powered on and        */
// /*  not every time that the robot is disabled.                               */
// /*---------------------------------------------------------------------------*/

// void pre_auton(void) {

//   // All activities that occur before the competition starts
//   // Example: clearing encoders, setting servo positions, ...
// }

// /*---------------------------------------------------------------------------*/
// /*                                                                           */
// /*                              Autonomous Task                              */
// /*                                                                           */
// /*  This task is used to control your robot during the autonomous phase of   */
// /*  a VEX Competition.                                                       */
// /*                                                                           */
// /*  You must modify the code to add your own robot specific commands here.   */
// /*---------------------------------------------------------------------------*/

// void autonomous(void) {
//   // ..........................................................................
//   // Insert autonomous user code here.
//   // ..........................................................................
// }

// /*---------------------------------------------------------------------------*/
// /*                                                                           */
// /*                              User Control Task                            */
// /*                                                                           */
// /*  This task is used to control your robot during the user control phase of */
// /*  a VEX Competition.                                                       */
// /*                                                                           */
// /*  You must modify the code to add your own robot specific commands here.   */
// /*---------------------------------------------------------------------------*/

// void usercontrol(void) {
//   // User control code here, inside the loop
//   while (1) {
//     // This is the main execution loop for the user control program.
//     // Each time through the loop your program should update motor + servo
//     // values based on feedback from the joysticks.

//     // ........................................................................
//     // Insert user code here. This is where you use the joystick values to
//     // update your motors, etc.
//     // ........................................................................

//     wait(20, msec); // Sleep the task for a short amount of time to
//                     // prevent wasted resources.
//   }
// }

// //
// // Main will set up the competition functions and callbacks.
// //
// int main() {
//   // Set up callbacks for autonomous and driver control periods.
//   Competition.autonomous(autonomous);
//   Competition.drivercontrol(usercontrol);

//   // Run the pre-autonomous function.
//   pre_auton();

//   // Prevent main from exiting with an infinite loop.
//   while (true) {
//     wait(100, msec);
//   }
// }



/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Clawbot Competition Template                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// L1                   motor         12              
// L2                   motor         13              
// L3                   motor         14              
// R1                   motor         18              
// R2                   motor         19              
// R3                   motor         20              
// Intake               motor         2               
// Export               motor         3               
// IMU                  inertial      10              
// Intake2              motor         1               
// front_panel          digital_out   A               
// Double_hook          digital_out   B               
// Lift_it_up           digital_out   H               
// x                    rotation      8               
// y                    rotation      15              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <string>

using namespace vex;
competition Competition;

int t = 0;
std::string mode = "";

void pre_auton(void) {
  vexcodeInit();
  
}

int main() {
   //角度初始化
  Controller1.Screen.clearScreen();
  Brain.Screen.clearScreen();
  IMU.calibrate();
  while(IMU.isCalibrating()){
    Brain.Screen.setCursor(5,5);
    Controller1.Screen.setCursor(2,4);
    Controller1.Screen.print("Inertial is calibrate");
    Brain.Screen.print("Inertial is calibrate");
    wait(50, msec);
  }
  thread Inertialinit(init);
  Competition.autonomous(Left);
  Competition.drivercontrol(usercontrol);
  // if (t){
  //   test();
  // }
  // mode = "left";
  // if (mode == "left"){
  //   Competition.autonomous(Left);
  //   Competition.drivercontrol(usercontrol);
  // }
  // if (mode == "right"){
  //   Competition.autonomous(Right);
  //   Competition.drivercontrol(usercontrol);
  // }
  // if (mode == "auto"){
  //   Competition.autonomous(Auto);
  //   Competition.drivercontrol(usercontrol);
  // }

  pre_auton();
  while (true) {
    wait(100, msec);
  }
}
