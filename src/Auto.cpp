#include "vex.h"

/**
 * 左侧自动程序（从场地左侧出发）
 */
void stoplr() {
  L.stop(brake);
  R.stop(brake);
}

void initcar() {
  Export.setStopping(hold);
  L.setStopping(brake);
  R.setStopping(brake);
}

void start2picked(bool r = 0) {
  // straight
  Intake.spin(forward, 100, vex::velocityUnits::pct);
  Intake2.spin(forward, 70, vex::velocityUnits::pct);
  MoveDistancePID(fwd, 8.8, 50, 20, 0, 0.2, 0.5); //23

  // turn
  if (!r)
    R.spinFor(forward, 1.1, turns, 20, vex::velocityUnits::pct);
  else
    L.spinFor(forward, 1.3, turns, 80, vex::velocityUnits::pct);
  stoplr();
  wait(50, msec);

  // pick
  if (!r) {
    MoveDistancePID(fwd, 16, 15, 10, 336, 0.2, 0.5);
    MoveDistancePID(reverse, 2, 15, 10, 336, 0.2, -0.5);
  } else {
    MoveDistancePID(fwd, 16, 15, 10, 24, 0.2, 0.5);
    MoveDistancePID(reverse, 2.7, 15, 10, 24, 0.2, -0.5);
  }
}

void picked2centered(bool r = 0) {
  // face at
  if (!r)
    smartTurn(225, 0.47, 0.05, 0.01);
  else
    smartTurn(315, 0.47, 0.05, 0.01);
  if (!r)
    MoveDistancePID(reverse, 14.2, 40, 10, 225, 0.2, -0.5);
  else
    MoveDistancePID(fwd, 14, 40, 10, 315, 0.4, 0.3);

  // 退后顶中桥吐球
  if (!r) {
    L.spin(reverse, 8, vex::velocityUnits::pct);
    R.spin(reverse, 8, vex::velocityUnits::pct);
  } else {
    // L.spin(fwd,8,vex::velocityUnits::pct);
    // R.spin(fwd,8,vex::velocityUnits::pct);
  }
  if (!r) {
    Intake2.stop(); // 关闭intake2，防止多余球入中桥
    Export.spin(reverse, 70, vex::velocityUnits::pct);
  } else {
    Intake2.spin(reverse, 100, vex::velocityUnits::pct);
    wait(500, msec);
    Intake.spin(reverse, 30, vex::velocityUnits::pct);
    wait(300, msec);
    Intake.spin(reverse, 10, vex::velocityUnits::pct);
  }
  stoplr();
  wait(500, msec);
  Export.stop();
  Intake.stop();
  Intake2.stop();
  if (r)
    wait(500, msec);
}

void centered2sided(bool r = 0) {
  if (!r)
    MoveDistancePID(fwd, 49.5, 50, 15, 225, 0.2, 0.5);
  else {
    Intake.spin(forward, 20, vex::velocityUnits::pct);
    L.spinFor(reverse, 0.3183098862, turns, 80, vex::velocityUnits::pct);
    R.spinFor(reverse, 0.3183098862, turns, 80, vex::velocityUnits::pct);
    // MoveDistancePID(reverse, 2, 50, 50, 315, 0.2, 0.7);
    // Intake2.spin(forward, 70, vex::velocityUnits::pct);
    // wait(200, msec);
    MoveDistancePID(reverse, 42.5, 50, 15, 315, 0.2, 0.3);
    // wait(500, msec);
  }
  smartTurn(180, 0.48, 0.05, 0.16);
  if (!r)
    Bucket_to_Bridge();
  else
    Bucket_to_Bridge_rdiff();
}

void knock_bucket() {
  move(fwd, 300, 100);
  moveTime(reverse, 100, 400);
}

void Left() {
  initcar();
  start2picked();
  picked2centered();
  centered2sided();
  knock_bucket();
}

void Right() {
  initcar();
  start2picked(1);
  picked2centered(1);
  centered2sided(1);
  knock_bucket();
}

void go_straight() {
  Intake.spin(forward, 100, vex::velocityUnits::pct);
  Intake2.spin(forward, 60, vex::velocityUnits::pct);
  // MoveDistancePID(fwd, 23, 50, 20, 0, 0.2, 0.5);
  // R.spinFor(forward, 1.1, turns, 20, vex::velocityUnits::pct);
  MoveDistancePID(fwd, 39, 60, 60, 345, 0.2, 0.5);
  MoveDistancePID(fwd, 60, 60, 50, 15, 0.2, 0.5);
  Intake.stop();
  Intake2.stop();
}

void rotation_to_correct(vex::directionType dir, float Speed, float targetAngle, float deltaDeg) {
  int deg_error = targetAngle - IMU.heading();
  while(deg_error >= deltaDeg) {
    deg_error = targetAngle - IMU.heading();
    L.spin(dir, Speed, pct);
    R.spin(dir, Speed, pct);
  }
  stoplr();
}

void go_side() {
  smartTurn(55, 0.47, 0.05, 0.01);
  // MoveDistancePID(reverse, 33, 30, 10, 55, 0.2, 0.4);
  rotation_to_correct(reverse, 30, 0, 2);
  return;
  Bucket_to_Bridge();
}

void only_bridge(){
  moveTime(reverse, 50, 1200);
  Export.spin(fwd, 100, vex::velocityUnits::pct);
  Intake.spin(forward, 100, vex::velocityUnits::pct);
  Intake2.spin(forward, 100, vex::velocityUnits::pct);
  for (int i = 0; i < 4; i++) {
    L.spinFor(fwd, 50, deg, 100, vex::velocityUnits::pct, false);
    R.spinFor(reverse, 90, deg, 100, vex::velocityUnits::pct);
    L.spinFor(reverse, 90, deg, 100, vex::velocityUnits::pct, false);
    R.spinFor(fwd, 50, deg, 100, vex::velocityUnits::pct);
  }
  moveTime(reverse, 100, 400);
  wait(0.6, sec);
  Intake.stop();
  Intake2.stop();
  Export.stop();
}

void picked2sided(){
  smartTurn(225, 0.47, 0.05, 0.01);
  MoveDistancePID(fwd, 35.3, 50, 15, 225, 0.2, 0.5);
  smartTurn(180, 0.47, 0.05, 0.01);
  only_bridge();
}

void Auto() {
  skill();
  return;
  initcar();
  start2picked();
  picked2sided();
  // go_straight();
  // go_side();
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
  Intake2.spin(forward,100,vex::velocityUnits::pct);
  Export.spin(forward,100,vex::velocityUnits::pct);
}


float Imu_calibrate_startstime=0;
void skill(){
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