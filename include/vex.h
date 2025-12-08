/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#pragma once
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"
#include "variable&funtion.h"

#define waitUntil(condition)                                                   \
    do {                                                                       \
        wait(5, msec);                                                         \
    } while (!(condition))

#define repeat(iterations)                                                     \
    for (int iterator = 0; iterator < iterations; iterator++)

// 点结构体
struct Point {
    double x, y;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
    Point operator+(const Point &p) const { return Point(x + p.x, y + p.y); }
    Point operator-(const Point &p) const { return Point(x - p.x, y - p.x); }
    double cot() const { return atan2(y, x); }
};

inline Point operator*(double k, const Point &p) {
    return Point(k * p.x, k * p.y);
}

// 位姿结构体
struct Pose {
    double x, y, theta;
    Point point() const { return Point(x, y); }
    Pose(double x = 0, double y = 0, double theta = 0)
        : x(x), y(y), theta(theta) {}
};
