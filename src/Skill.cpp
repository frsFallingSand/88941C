// #include "vex.h"

// void imu_init(){
//   while(IMU.isCalibrating()){
//     Brain.Screen.setCursor(5,5);
//     Controller1.Screen.setCursor(2,4);
//     Controller1.Screen.print("Inertial is calibrate");
//     Brain.Screen.print("Inertial is calibrate");
//     wait(50, msec);
//   }
// }

// void intakefuntion(){
//   Export.stop(hold);
//   Export.spin(fwd,0.5,vex::velocityUnits::pct);
//   Intake.spin(forward,100,vex::velocityUnits::pct);
//   Intake2.spin(forward,65,vex::velocityUnits::pct);
// }

// void intakestop(){
//   Intake.stop();
//   Intake2.stop();
//   Export.stop();
// }

// void intake_import(){
//   Export.stop(coast);
//   //Export.spin(forward,100,vex::velocityUnits::pct);
//   Intake2.spin(reverse,100,vex::velocityUnits::pct);//反转
//   Export.spin(forward,100,vex::velocityUnits::pct);
//   wait(70,msec);
//   Intake2.stop();
//   Intake2.spin(forward,70,vex::velocityUnits::pct);
//   Intake.spin(forward,100,vex::velocityUnits::pct);
// }

// void moveStop(){
//   L.stop();
//   R.stop();
// }

// void Skill(){
//   float times = Brain.Timer.value();
//   intakefuntion();
//   MoveDistancePID(fwd,10,30,20,0,0.2,0.2);
//   R.spinFor(forward,1.2,turns,30,vex::velocityUnits::pct);
//   L.stop(brake);
//   R.stop(brake);
//   wait(50,msec);
//   MoveDistancePID(fwd,13,12,12,331,0.2,0.2);
//   //MoveDistancePID(reverse,3.5,30,10,331,0.2,-0.2);
//   smartTurn(225 , 0.48 , 0.03, 0.01);
//   MoveDistancePID(fwd,26.5,50,50,225,0.2,0.2);
//   smartTurn(180 , 0.48 , 0.03, 0.01);
//   //MoveDistancePID(reverse,7,50,50,180,0.2,0.2);
//   moveTime(reverse,30,600);
//   Export.setStopping(coast);
//   intake_import();
//   //转动后退，确保卡到长桥
//   for(int i=0;i<2;i++){
//     L.spinFor(fwd,40,deg,100,vex::velocityUnits::pct,false);
//     R.spinFor(reverse,80,deg,100,vex::velocityUnits::pct);
//     L.spinFor(reverse,80,deg,100,vex::velocityUnits::pct,false);
//     R.spinFor(fwd,40,deg,100,vex::velocityUnits::pct);
//   }
//   wait(0.6,sec);
//   front_panel.set(true);
//   //退后顶桥
//   moveTime(reverse,100,200);
//   intakestop();
//   intakefuntion();
//   MoveDistancePID(fwd,17,60,50,180,0.2,0.2);
//   moveTime(fwd,30,700);
//   //拿1桶
//   wait(0.8,sec);
//   move(reverse,80,10);
//   moveTime(fwd,20,200);

//   MoveDistancePID(reverse,5,60,50,180,0.2,-0.2);
//   front_panel.set(false);
//   L.spinFor(reverse,700,deg,40,vex::velocityUnits::pct);
//   L.stop(brake);
//   R.stop(brake);
//   wait(50,msec);
//   L.spinFor(reverse,600,deg,50,vex::velocityUnits::pct,false);
//   R.spinFor(reverse,1050,deg,50,vex::velocityUnits::pct);
//
//   //退后到2桶的位置
//   intakestop();
//   MoveDistancePID(reverse,66,80,60,180,0.2,-0.5);
//   smartTurn(-90 , 0.49 , 0.03, 0.01);

//
//   //靠墙定位，陀螺仪改0
//   moveTime(fwd,30,500);
//   wait(100,msec);
//   IMU.setHeading(0,deg);
//   wait(50,msec);
//   MoveDistancePID(reverse,12.5,40,20,0,0.2,-0.4);
//   intakestop();
//   smartTurn(90 , 0.49 , 0.03, 0.01);
//   wait(100,msec);
//   moveTime(reverse,50,800);
//   Export.setStopping(coast);
//   //导入球
//   intake_import();
//   //转动后退，确保卡到长桥
//   for(int i=0;i<2;i++){
//     L.spinFor(fwd,40,deg,100,vex::velocityUnits::pct,false);
//     R.spinFor(reverse,80,deg,100,vex::velocityUnits::pct);
//     L.spinFor(reverse,80,deg,100,vex::velocityUnits::pct,false);
//     R.spinFor(fwd,40,deg,100,vex::velocityUnits::pct);
//   }
//   moveTime(reverse,100,300);
//   wait(1,sec);
//   front_panel.set(true);
//   moveTime(reverse,100,300);
//   // //退后顶桥
//   intakestop();
//   intakefuntion();

//   MoveDistancePID(fwd,17,60,50,90,0.2,0.2);
//   moveTime(fwd,30,750);
//   //拿2桶
//   wait(1,sec);
//   move(reverse,80,10);
//   moveTime(fwd,20,200);

//   MoveDistancePID(reverse,17,60,50,90,0.2,-0.2);
//   moveTime(reverse,30,650);
//   intake_import();
//   wait(2,sec);
//   moveStop();
//   front_panel.set(false);
//   //Expor回转一下，在正转，让最后一个球导入
//   // Export.spinFor(reverse,100,deg,50,vex::velocityUnits::pct);
//   // Export.spin(fwd,100,vex::velocityUnits::pct);
//   moveTime(reverse,100,200);
//   wait(100,msec);
//   intakestop();

//
//   MoveDistancePID(fwd,13,50,40,90,0.2,0.2);
//   intakefuntion();
//   smartTurn(215 , 0.49 , 0.03, 0.01);
//   MoveDistancePID(fwd,35,40,30,215,0.2,0.2);
//   smartTurn(165 , 0.49 , 0.03, 0.01);
//   MoveDistancePID(fwd,77,80,70,160,0.2,0.2);
//   //smartTurn(180 , 0.48 , 0.03, 0.01);
//   //前进过去已经接近90度，没有不要在走一此转向

//   moveTime(fwd,30,400);
//   wait(50,msec);
//   IMU.setHeading(0,deg);
//   wait(50,msec);
//   MoveDistancePID(reverse,12.5,40,20,0,0.2,-0.4);
//   intakestop();
//   smartTurn(-90 , 0.49 , 0.03, 0.01);
//   wait(100,msec);
//   moveTime(reverse,60,700);
//   Export.setStopping(coast);
//   //导入球
//   intake_import();
//   //转动后退，确保卡到长桥
//   for(int i=0;i<2;i++){
//     L.spinFor(fwd,40,deg,100,vex::velocityUnits::pct,false);
//     R.spinFor(reverse,80,deg,100,vex::velocityUnits::pct);
//     L.spinFor(reverse,80,deg,100,vex::velocityUnits::pct,false);
//     R.spinFor(fwd,40,deg,100,vex::velocityUnits::pct);
//   }
//   moveTime(reverse,100,300);
//   wait(0.8,sec);
//   front_panel.set(true);
//   // //退后顶桥
//   intakestop();
//   intakefuntion();

//   MoveDistancePID(fwd,17,60,50,270,0.2,0.2);
//   moveTime(fwd,30,700);
//   //拿3桶
//   wait(0.8,sec);
//   move(reverse,80,10);
//   moveTime(fwd,20,200);

//   MoveDistancePID(reverse,6,60,50,270,0.2,-0.2);
//   front_panel.set(false);
//   L.spinFor(reverse,650,deg,40,vex::velocityUnits::pct);
//   L.stop(brake);
//   R.stop(brake);
//   wait(50,msec);
//   L.spinFor(reverse,550,deg,50,vex::velocityUnits::pct,false);
//   R.spinFor(reverse,1050,deg,50,vex::velocityUnits::pct);
//
//   //退后到4桶的位置
//   intakestop();
//   MoveDistancePID(reverse,68,80,50,270,0.2,-0.5);
//   smartTurn(0 , 0.49 , 0.03, 0.01);

//
//   //靠墙定位，陀螺仪改0
//   moveTime(fwd,30,500);
//   wait(100,msec);
//   IMU.setHeading(0,deg);
//   wait(100,msec);
//   MoveDistancePID(reverse,12.5,40,20,0,0.2,-0.4);
//   smartTurn(90 , 0.49 , 0.03, 0.01);
//   wait(100,msec);
//   moveTime(reverse,60,800);

//   Export.setStopping(coast);
//   //导入球
//   intake_import();
//   //转动后退，确保卡到长桥
//   for(int i=0;i<3;i++){
//     L.spinFor(fwd,40,deg,100,vex::velocityUnits::pct,false);
//     R.spinFor(reverse,80,deg,100,vex::velocityUnits::pct);
//     L.spinFor(reverse,80,deg,100,vex::velocityUnits::pct,false);
//     R.spinFor(fwd,40,deg,100,vex::velocityUnits::pct);
//   }
//   moveTime(reverse,100,200);
//   wait(1,sec);
//   front_panel.set(true);
//   // //退后顶桥
//   intakestop();
//   intakefuntion();

//   MoveDistancePID(fwd,17,60,50,90,0.2,0.2);
//   moveTime(fwd,30,650);
//   //拿4桶
//   wait(1,sec);
//   move(reverse,80,10);
//   moveTime(fwd,20,200);

//   MoveDistancePID(reverse,17,60,50,90,0.2,-0.2);
//   moveTime(reverse,50,650);
//   for(int i=0;i<2;i++){
//     L.spinFor(fwd,30,deg,100,vex::velocityUnits::pct,false);
//     R.spinFor(reverse,80,deg,100,vex::velocityUnits::pct);
//     L.spinFor(reverse,80,deg,100,vex::velocityUnits::pct,false);
//     R.spinFor(fwd,30,deg,100,vex::velocityUnits::pct);
//   }
//   intake_import();
//   wait(2,sec);
//   moveTime(reverse,100,200);
//   ////Expor回转一下，在正转，让最后一个球导入
//   // Export.spinFor(reverse,100,deg,50,vex::velocityUnits::pct);
//   // Export.spin(fwd,100,vex::velocityUnits::pct);
//   wait(0.2,sec);
//   intakestop();
//   moveStop();
//   front_panel.set(false);
//   moveTime(reverse,100,300);

//   //吐空，贴边靠近停泊区
//   intakefuntion();
//   Export.spin(reverse,100,vex::velocityUnits::pct);
//   L.spinFor(fwd,3000,deg,65,vex::velocityUnits::pct,false);
//   R.spinFor(fwd,2100,deg,40,vex::velocityUnits::pct);
//   wait(50,msec);
//   move(fwd,150,60);
//   intakestop();
//   moveStop();
//   //停车
//   intakefuntion();
//   Export.spin(reverse,100,vex::velocityUnits::pct);
//   front_panel.set(true);
//   wait(200,msec);
//   move(fwd,1800,60);
//   front_panel.set(false);
//   //intakestop();
//   moveStop();
//

//   Controller1.Screen.clearScreen();
//   Controller1.Screen.setCursor(2,4);
//   Controller1.Screen.print(Brain.Timer.value()-times);
//   wait(5,sec);
// }
