#include "vex.h"

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
  Intake2.spin(forward,100,vex::velocityUnits::pct);
  Export.spin(forward,100,vex::velocityUnits::pct);
}


void Skill(){
    float Imu_calibrate_startstime=0;
  wait(3,sec);
  Intake.spin(forward,100,vex::velocityUnits::pct);
  Intake2.spin(forward,60,vex::velocityUnits::pct);
  MoveDistancePID(fwd,11,35,20,0,0.2,0.2);
  R.spinFor(forward,1.2,turns,20,vex::velocityUnits::pct);
  L.stop(brake);
  R.stop(brake);
  wait(50,msec);
  MoveDistancePID(fwd,14,15,15,331,0.2,0.2);
  MoveDistancePID(reverse,3.5,30,10,331,0.2,-0.2);
  smartTurn(225 , 0.48 , 0.07, 0.01);
  MoveDistancePID(fwd,34,55,15,225,0.2,0.4);
  intakestop();
  smartTurn(180 , 0.48 , 0.05 , 0.16);
  wait(100,msec);
  Intake2.spinFor(reverse,100,deg,100,vex::velocityUnits::pct,false);//反转防卡
  moveTime(reverse,70,500);
  L.spin(reverse,10,pct);
  R.spin(reverse,10,pct);
  intake_import();
  wait(1.2,sec);
  L.stop();
  R.stop();
  intakestop();
  front_panel.set(true);
  intakefuntion();
  MoveDistancePID(fwd,27.5,40,20,180,0.2,0.2);
  moveTime(fwd,30,500);
  wait(1,sec);

  MoveDistancePID(reverse,8,50,8,180,0.2,-0.5);
  front_panel.set(false);
  wait(50,msec);
  L.spinFor(reverse,700,deg,40,vex::velocityUnits::pct);
  L.stop(brake);
  R.stop(brake);
  wait(50,msec);
  L.spinFor(reverse,600,deg,40,vex::velocityUnits::pct,false);
  R.spinFor(reverse,1150,deg,40,vex::velocityUnits::pct);
  intakestop();
  MoveDistancePID(reverse,65,80,15,180,0.2,-0.5);
  R.spinFor(fwd,1000,deg,40,vex::velocityUnits::pct);
  //靠墙定位
  moveTime(reverse,30,600);
  wait(100,msec);
  move(fwd,950,30);
  L.spinFor(reverse,520,deg,40,vex::velocityUnits::pct,false);
  R.spinFor(fwd,520,deg,40,vex::velocityUnits::pct);
  wait(100,msec);
  moveTime(reverse,50,600);
  Intake2.spinFor(reverse,100,deg,100,vex::velocityUnits::pct,false);//反转防卡
  for(int i=0;i<2;i++){
    L.spinFor(fwd,40,deg,100,vex::velocityUnits::pct,false);
    R.spinFor(reverse,90,deg,100,vex::velocityUnits::pct);
    L.spinFor(reverse,90,deg,100,vex::velocityUnits::pct,false);
    R.spinFor(fwd,40,deg,100,vex::velocityUnits::pct);
  }
  moveTime(reverse,30,200);
  
  
  IMU.calibrate();
  intake_import();//导入
  Imu_calibrate_startstime = Brain.Timer.value();
  while(IMU.isCalibrating()){
    wait(10,msec);
  }
  //如果不足3秒，继续等到且导入
  if(Brain.Timer.value()-Imu_calibrate_startstime<=3){
    wait(Brain.Timer.value()-Imu_calibrate_startstime,msec);
  }

  move(fwd,200,20);
  moveTime(reverse,20,400);
  intakestop();

 
  //拿2桶
  front_panel.set(true);
  intakefuntion();
  MoveDistancePID(fwd,27,40,20,0,0.2,0.2);
  moveTime(fwd,30,400);
  wait(0.9,sec);
  intakestop();
  front_panel.set(false);
  MoveDistancePID(reverse,10,40,20,0,0.2,-0.2);
  smartTurn(135 , 0.48 , 0.07, 0.01);
  intakefuntion();
  MoveDistancePID(fwd,33,40,20,135,0.2,0.2);
  smartTurn(90 , 0.48 , 0.07, 0.01);
  MoveDistancePID(fwd,43,62,20,90,0.2,0.2);
  smartTurn(45 , 0.48 , 0.07, 0.01);
  MoveDistancePID(reverse,10,40,20,45,0.2,-0.2);
  Intake2.spin(reverse,100,vex::velocityUnits::pct);
  moveTime(reverse,20,500);
  //导入中桥
  intakefuntion();
  Export.spin(reverse,40,vex::velocityUnits::pct);
  wait(4,sec);

  intakestop();

  //去3桶位置靠墙定位
  R.spinFor(fwd,1000,deg,80,vex::velocityUnits::pct,false);
  L.spinFor(fwd,1600,deg,80,vex::velocityUnits::pct);
}