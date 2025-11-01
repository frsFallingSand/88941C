#include "vex.h"

void imu_init(){
  while(IMU.isCalibrating()){
    Brain.Screen.setCursor(5,5);
    Controller1.Screen.setCursor(2,4);
    Controller1.Screen.print("Inertial is calibrate");
    Brain.Screen.print("Inertial is calibrate");
    wait(50, msec);
  }
}

void intakefuntion(){
  Intake.spin(forward,100,vex::velocityUnits::pct);
  Intake2.spin(forward,100,vex::velocityUnits::pct);
}

void intakestop(){
  Intake.stop();
  Intake2.stop();
  Export.stop();
}

void intake_import(){
  Intake.spin(forward,100,vex::velocityUnits::pct);
  Intake2.spin(forward,70,vex::velocityUnits::pct);
  Export.spin(forward,100,vex::velocityUnits::pct);
}


void Skill(){
    imu_init();

  wait(1,sec);
  
  Intake.spin(forward,100,vex::velocityUnits::pct);
  Intake2.spin(forward,60,vex::velocityUnits::pct);
  MoveDistancePID(fwd,10.5,35,20,0,0.2,0.2);
  R.spinFor(forward,1.2,turns,40,vex::velocityUnits::pct);
  L.stop(brake);
  R.stop(brake);
  wait(50,msec);
  MoveDistancePID(fwd,14,30,20,333,0.2,0.2);
  wait(50,msec);
  MoveDistancePID(reverse,3.5,30,10,333,0.2,-0.2);
  //指向1桶
  smartTurn(225 , 0.49 , 0.019, 0.01);
  MoveDistancePID(fwd,34.5,55,20,225,0.2,0.4);
  wait(50,msec);
  intakestop();
  smartTurn(180 , 0.49 , 0.019 , 0.01);


  front_panel.set(true);
  //Intake2.spin(reverse,100,pct);
  wait(50,msec);
  //Intake2.stop();
  intakefuntion();
  moveTime(fwd,40,500);
  L.spin(fwd,2,pct);
  R.spin(fwd,2,pct);
  wait(1.3,sec);
  L.stop(brake);
  R.stop(brake);
  wait(0.2,sec);
  MoveDistancePID(reverse,26,40,40,180,0.2,-0.4);
  Intake2.spin(reverse,100,pct);
  moveTime(reverse,70,200);
  intakestop();
  intake_import();
  for(int i=0;i<2;i++){
    L.spinFor(fwd,40,deg,100,vex::velocityUnits::pct,false);
    R.spinFor(reverse,50,deg,100,vex::velocityUnits::pct);
    L.spinFor(reverse,50,deg,100,vex::velocityUnits::pct,false);
    R.spinFor(fwd,40,deg,100,vex::velocityUnits::pct);
  }
  
  L.spin(reverse,100,pct);
  R.spin(reverse,100,pct);
  wait(2.2,sec);
  move(fwd,200,30);
  wait(100,msec);
  front_panel.set(false);
  moveTime(reverse,20,400);
  wait(100,msec);


  MoveDistancePID(fwd,15,50,8,180,0.2,-0.5);
  wait(50,msec);
  L.spinFor(reverse,600,deg,40,vex::velocityUnits::pct);
  L.stop(brake);
  R.stop(brake);
  wait(50,msec);
  L.spinFor(reverse,450,deg,40,vex::velocityUnits::pct,false);
  R.spinFor(reverse,1000,deg,40,vex::velocityUnits::pct);
  intakestop();
  MoveDistancePID(reverse,74,80,15,180,0.2,-0.5);
  R.spinFor(fwd,1400,deg,40,vex::velocityUnits::pct);
  //靠墙定位
  moveTime(reverse,40,700);
  wait(200,msec);
  IMU.setHeading(0,deg);
  wait(100,msec);
  MoveDistancePID(fwd,14,40,15,0,0.2,0.5);
  smartTurn(-90 , 0.48 , 0.018 , 0.01);

  // //拿2桶
  front_panel.set(true);
  wait(100,msec);
  intakefuntion();
  MoveDistancePID(fwd,10,40,20,270,0.2,0.2);
  moveTime(fwd,15,800);
  wait(1.3,sec);
  intakestop();
  front_panel.set(false);
  MoveDistancePID(reverse,9,40,20,270,0.2,-0.2);
  smartTurn(45 , 0.48 , 0.018, 0.01);
  intakefuntion();
  MoveDistancePID(fwd,32,50,30,45,0.2,0.3);
  smartTurn(0 , 0.48 , 0.018, 0.01);
  MoveDistancePID(fwd,38.5,70,30,0,0.2,0.3);
  smartTurn(315 , 0.48 , 0.018, 0.01);
  MoveDistancePID(reverse,8,40,20,315,0.2,-0.2);
  Intake2.spin(reverse,100,vex::velocityUnits::pct);
  moveTime(reverse,20,600);
  //导入中桥
  intakefuntion();
  Export.spin(reverse,40,vex::velocityUnits::pct);
  wait(1,sec);
  Export.spin(reverse,30,vex::velocityUnits::pct);
  wait(1.5,sec);
  wait(200,msec);

  R.spinFor(fwd,1600,deg,80,vex::velocityUnits::pct,false);
  L.spinFor(fwd,2200,deg,80,vex::velocityUnits::pct);  
  intakestop();
  moveTime(fwd,40,1200);
  wait(100,msec);
  IMU.setHeading(0,deg);
  wait(100,msec);

  //去3桶位置
  MoveDistancePID(reverse,9,55,15,0,0.2,-0.4);
  smartTurn(270 , 0.47 , 0.018 , 0.16);
  front_panel.set(true);
  wait(100,msec);
  intakefuntion();
  moveTime(fwd,40,700);
  // L.spin(fwd,2,pct);
  // R.spin(fwd,2,pct);
  wait(1.2,sec);
  L.stop(brake);
  R.stop(brake);
  wait(0.2,sec);
  MoveDistancePID(reverse,5,30,15,270,0.2,-0.5);

  front_panel.set(false);
  wait(100,msec);
  
  L.spinFor(reverse,450,deg,40,vex::velocityUnits::pct);
  wait(100,msec);
  L.spinFor(reverse,700,deg,40,vex::velocityUnits::pct,false);
  R.spinFor(reverse,1200,deg,40,vex::velocityUnits::pct);  
  intakestop();

  MoveDistancePID(reverse,76,80,15,270,0.2,-0.5);
  R.spinFor(fwd,1000,deg,40,vex::velocityUnits::pct);
  // //靠墙定位
  moveTime(reverse,40,700);
  wait(100,msec);
  IMU.setHeading(0,deg);
  wait(100,msec);
  //四桶

  MoveDistancePID(fwd,12,30,15,0,0.2,0.5);
  smartTurn(270 , 0.49 , 0.02 , 0.16);
  moveTime(reverse,60,700);
  intake_import();
  for(int i=0;i<2;i++){
    L.spinFor(fwd,40,deg,100,vex::velocityUnits::pct,false);
    R.spinFor(reverse,50,deg,100,vex::velocityUnits::pct);
    L.spinFor(reverse,50,deg,100,vex::velocityUnits::pct,false);
    R.spinFor(fwd,40,deg,100,vex::velocityUnits::pct);
  }
  L.spin(reverse,100,vex::velocityUnits::pct);
  R.spin(reverse,100,vex::velocityUnits::pct);
  wait(1.2,sec);
  L.stop();
  R.stop();

  intakestop();
  front_panel.set(true);
  intakefuntion();
  MoveDistancePID(fwd,24,50,20,270,0.2,0.4);
  moveTime(fwd,20,600);
  wait(0.9,sec);

  MoveDistancePID(reverse,26,40,20,270,0.2,-0.4);
  Intake2.spin(reverse,100,pct);
  moveTime(reverse,70,200);
  intakestop();
  intake_import();
  for(int i=0;i<2;i++){
    L.spinFor(fwd,40,deg,100,vex::velocityUnits::pct,false);
    R.spinFor(reverse,50,deg,100,vex::velocityUnits::pct);
    L.spinFor(reverse,50,deg,100,vex::velocityUnits::pct,false);
    R.spinFor(fwd,40,deg,100,vex::velocityUnits::pct);
  }
//   DigitalOutF.set(true);
  front_panel.set(false);
  L.spin(reverse,100,pct);
  R.spin(reverse,100,pct);
  wait(2,sec);
  
  L.spin(fwd,60,pct);
  R.spin(fwd,40,pct);
  wait(1.5,sec);
  L.spin(fwd,100,pct);
  R.spin(fwd,100,pct);
  wait(0.5,sec);
  L.stop();
  R.stop();
}