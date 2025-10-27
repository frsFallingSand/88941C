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
  MoveDistancePID(fwd, 23, 50, 20, 0, 0.2, 0.5);

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
  MoveDistancePID(fwd, 23, 50, 20, 0, 0.2, 0.5);
  R.spinFor(forward, 1.1, turns, 20, vex::velocityUnits::pct);
  MoveDistancePID(fwd, 16, 15, 10, 336, 0.2, 0.5);
  MoveDistancePID(fwd, 71, 80, 50, 0, 0.2, 0.5);
  Intake.stop();
  Intake2.stop();
}

void go_side() {
  smartTurn(45, 0.47, 0.05, 0.01);
  MoveDistancePID(reverse, 33, 15, 10, 336, 0.2, 0.5);
  Bucket_to_Bridge();
  knock_bucket();
}

void Auto() {
  initcar();
  go_straight();
  go_side();
}
