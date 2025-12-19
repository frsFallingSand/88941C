// #include "vex.h"

// void intake_start(){
//   Intake.spin(forward,100,vex::velocityUnits::pct);
//   Intake2.spin(forward,70,vex::velocityUnits::pct);
// }

// void lr_brake(){
//   L.stop(brake);
//   R.stop(brake);
// }

// void lr_stop(){
//   L.stop();
//   R.stop();
// }

// void Left(){
//   Export.stop(hold);
//   intake_start();
//   //Export.spin(reverse,70,vex::velocityUnits::pct);

//   // go
//   MoveDistancePID(fwd,9,30,20,0,0.2,0.2);
//   R.spinFor(forward,1.2,turns,20,vex::velocityUnits::pct);
//   lr_brake();
//   wait(50,msec);

//   // pick
//   MoveDistancePID(fwd,8.5,12,12,331,0.2,0.2);
//   //MoveDistancePID(reverse,4,30,10,331,0.2,-0.2);

//   ///

//   // go to center
//   smartTurn(225 , 0.48 , 0.03, 0.01);
//   MoveDistancePID(reverse,13.5,40,10,224,0.2,-0.3);

//   // push center
//   Export.setStopping(coast);
//   Intake2.stop();//关闭intake2，防止多余球入中桥
//   Export.spin(reverse,100,vex::velocityUnits::pct);
//   //退后顶中桥吐球
//   L.spin(reverse,5,vex::velocityUnits::pct);
//   R.spin(reverse,5,vex::velocityUnits::pct);
//   wait(250,msec);
//   Export.stop();
//   lr_brake();

//   ///

//   // go to side
//   // push side
//   MoveDistancePID(fwd,49,55,15,226,0.2,0.4);
//   // wait(100,msec);
//   smartTurn(180 , 0.49 , 0.03 , 0.16);
//   wait(100,msec);
//   Bucket_to_Bridge();
//   // DigitalOutF.set(true);
//   Occupying_the_scoring_zone();
//   lr_stop();

//   // 要求：停止时位于正方向
// }

// void Right(){
//   Export.stop(hold);
//   intake_start();
//   //Export.spin(reverse,70,vex::velocityUnits::pct);

//   // go
//   MoveDistancePID(fwd,9,30,20,0,0.2,0.2);
//   L.spinFor(forward,1.2,turns,20,vex::velocityUnits::pct);
//   lr_brake();
//   wait(50,msec);

//   // pick
//   MoveDistancePID(fwd,9.6,12,12,29,0.2,0.2);
//   //MoveDistancePID(reverse,6,30,10,29,0.2,-0.2);
//   Intake.setStopping(hold);
//   Intake2.spin(reverse,60,vex::velocityUnits::pct);

//   ///

//   // go to center
//   smartTurn(-45 , 0.48 , 0.03, 0.01);
//   MoveDistancePID(fwd,13,40,10,-46,0.2,0.3);
//   lr_brake();

//   // push center
//   Intake2.stop();
//   Intake.spinFor(reverse,250,deg,30,vex::velocityUnits::pct);
//   Intake2.stop();//关闭intake2，防止多余球入中桥
//   Intake.setStopping(coast);

//   ///

//   // go to side
//   // push side
//   MoveDistancePID(reverse,42,55,15,-46,0.2,-0.4);
//   smartTurn(180 , 0.48 , 0.05 , 0.16);
//   Intake.spin(forward,100,vex::velocityUnits::pct);
//   Intake2.spin(forward,70,vex::velocityUnits::pct);
//   Bucket_to_Bridge();
//   // DigitalOutF.set(true);
//   Occupying_the_scoring_zone();
//   lr_stop();
// }
