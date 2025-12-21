#pragma once
#include "class/d.h"
#include "curve.cpp"
#include "vex.h"
#include "vex_global.h"
#include <class/bezier.hpp>
#include <class/ppcontrol.hpp>
#include <cmath>

ppc::Builder &ppc::Builder ::max(double v) {
    _max = v;
    return *this;
}
ppc::Builder &ppc::Builder ::min(double v) {
    _min = v;
    return *this;
}
ppc::Builder &ppc::Builder ::path(const std::vector<Point> &p) {
    _path = p;
    return *this;
}
ppc::Builder &ppc::Builder ::path(const class Bezier &p) {
    _path = p.getPath();
    return *this;
}
ppc::Builder &ppc::Builder ::backward(bool a) {
    _backward = a;
    return *this;
}
ppc::Builder &ppc::Builder ::lookahead(double a) {
    _lookahead = a;
    return *this;
}
ppc::Builder &ppc::Builder ::kp(double k) {
    _kp = k;
    return *this;
}
ppc::Builder &ppc::Builder ::i(int a) {
    _i = a;
    return *this;
}
ppc::Builder &ppc::Builder ::r(double a) {
    _r = a;
    return *this;
}
ppc ppc::Builder::build() const {
    return ppc(_max, _min, _path, _backward, _lookahead, _kp, _r, _tpr, _width,
               _i);
}

ppc::ppc(double max, double min, const std::vector<Point> &p, bool bw, double l,
         double kp, double r, double tpr, double width, int i)
    : _max(max), _min(min), _path(p), _backward(bw), _lookahead(l), _kp(kp),
      _r(r), _tpr(tpr), _width(width), _i(i) {}

// ↑ MANUL BUILDER

std::vector<Point> ppc::getPath() { return _path; }

double ppc::normAngle(double a) {
    while (a > M_PI)
        a -= 2 * M_PI;
    while (a < -M_PI)
        a += 2 * M_PI;
    return a;
}

void ppc::setup() {
    // WARNING: Ensure you have init IMU

    x.resetPosition();
    y.resetPosition();
    lastX = x.position(degrees);
    lastY = y.position(degrees);

    p.theta = IMU.rotation(degrees) * M_PI / 180.0;
}

void ppc::update() {
    // // 当前X,Y
    // double curX = x.position(degrees);
    // double curY = y.position(degrees);
    // // X,Y增量
    // double dX = curX - lastX;
    // double dY = curY - lastY;
    // // 转换为45度方向,cos，sin传入弧度值，非角度值
    // double dx45 =
    //     ((dX * cos(M_PI / 4) - dY * sin(M_PI / 4)) / _tpr) * (2 * M_PI * _r);
    // double dy45 =
    //     ((dX * sin(M_PI / 4) + dY * cos(M_PI / 4)) / _tpr) * (2 * M_PI * _r);
    // // 度转弧
    // double curT = IMU.rotation(degrees) * M_PI / 180.0;
    // // 转为场地坐标
    // double gdX = dx45 * cos(p.theta) - dy45 * sin(p.theta);
    // double gdY = dx45 * sin(p.theta) + dy45 * cos(p.theta);

    // p.x += gdX;
    // p.y += gdY;
    // p.theta = curT;

    // lastX = curX;
    // lastY = curY;
    double curX = x.position(degrees);
    double curY = y.position(degrees);
    double dX = curX - lastX;
    double dY = curY - lastY;

    // 获取当前IMU航向（这个时刻的）
    double current_theta = IMU.rotation(degrees) * M_PI / 180.0;

    // 使用平均航向（更准确）
    double avg_theta = (p.theta + current_theta) / 2.0;

    // 45度编码器转换（保持不变）
    double dx45 =
        ((dX * cos(M_PI / 4) - dY * sin(M_PI / 4)) / _tpr) * (2 * M_PI * _r);
    double dy45 =
        ((dX * sin(M_PI / 4) + dY * cos(M_PI / 4)) / _tpr) * (2 * M_PI * _r);

    // 使用平均航向转换到场地坐标
    double gdX = dx45 * cos(avg_theta) - dy45 * sin(avg_theta);
    double gdY = dx45 * sin(avg_theta) + dy45 * cos(avg_theta);

    p.x += gdX;
    p.y += gdY;
    p.theta = current_theta; // 更新为当前IMU值

    lastX = curX;
    lastY = curY;

    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(p.x);
    Brain.Screen.setCursor(1, 10);
    Brain.Screen.print(p.y);
    // Brain.Screen.setCursor(2, 1);
    // Brain.Screen.print(p.theta);
    // Brain.Screen.setCursor(3, 1);
    // Brain.Screen.print("dX:%f", dX);
    // Brain.Screen.setCursor(3, 15);
    // Brain.Screen.print("dY:%f", dY);
    // // Brain.Screen.setCursor(6, 1);
    // // Brain.Screen.print("dx45:%f", dx45);
    // // Brain.Screen.setCursor(7, 1);
    // // Brain.Screen.print("dY45:%f", dy45);

    // Brain.Screen.setCursor(4, 1);
    // Brain.Screen.print("curX:%f-lastX:%f", curX, lastX);
    // Brain.Screen.setCursor(4, 20);
    // Brain.Screen.print("curY:%f-lastY:%f", curY, lastY);

    // Brain.Screen.setCursor(5, 1);
    // Brain.Screen.print("curT%f", curT);
}

int ppc::lookahead(int startI) {
    if (_path.empty() || startI >= _path.size())
        return -1;

    int closestI = startI;
    double minD = std::numeric_limits<double>::max();
    for (int i = startI; i < _path.size(); i++) {
        double d = Curve::distance(p.point(), _path[i]);
        if (d < minD) {
            minD = d;
            closestI = i;
        }
    }

    for (int i = closestI; i < _path.size(); i++) {
        double d = Curve::distance(p.point(), _path[i]);
        if (d >= _lookahead) {
            return i;
        }
    }

    return _path.size() - 1;
}

double linearToMotorRPM(double linear_speed) {
    // 1. 线速度 -> 轮子角速度 (rad/s)
    double wheel_angular = linear_speed / 1.625; // 轮子r

    // 2. 轮子角速度 -> 轮子转速 (RPM)
    double wheel_rpm = wheel_angular * 60.0 / (2.0 * M_PI);

    // 3. 考虑齿比得到电机转速
    double motor_rpm = wheel_rpm * 0.75; // 齿比

    return motor_rpm;
}

void ppc::control(int i) {
    if (_path.empty())
        return;

    // i = lookahead(i);
    if (i == -1)
        return;

    Point t = _path[i];
    double tH = Curve::tangent(_path, i);
    if (_backward)
        tH = normAngle(tH + M_PI);

    // t:target p:pose
    Point d = t - p.point(); // 场地坐标系误差
    double dx = d.x;
    double dy = d.y;

    // Point dr = Point(dx * cos(-p.theta) - dy * sin(-p.theta),
    //                  dx * sin(-p.theta) + dy * cos(-p.theta)); //
    //                  机器坐标系误差

    Point dr = Point(dx * cos(p.theta) + dy * sin(p.theta),
                     -dx * sin(p.theta) + dy * cos(p.theta)); // 机器坐标系误差

    double ddis = Curve::distance(dr, Point(0, 0));

    double k = (ddis > 0.5) ? -(2 * dr.x) / (ddis * ddis) : 0; // 曲率

    double He = normAngle(tH - p.theta);
    double Hc = He * _kp;
    // double Hc = 0;

    double disT = Curve::distance(p.point(), t);
    double baseV = _min + (_max - _min) * std::min(1.0, disT / 5.0);
    if (_backward)
        baseV = -baseV;

    // TODO: reverse check
    double lS = linearToMotorRPM(baseV * (1 - k * _width / 2)) + Hc;
    double rS = linearToMotorRPM(baseV * (1 + k * _width / 2)) - Hc;

    lS = clamp(lS, -_max, _max);
    rS = clamp(rS, -_max, _max);

    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print(lS);
    Brain.Screen.setCursor(6, 10);
    Brain.Screen.print(rS);
    Brain.Screen.setCursor(6, 20);
    Brain.Screen.print("ddis:%.f", ddis);

    // pos lv rv curT
    // Brain.Screen.setCursor(1, 1);
    // Brain.Screen.print("p: %d", p.x);
    // Brain.Screen.setCursor(1, 10);
    // Brain.Screen.print(p.y);
    // Brain.Screen.setCursor(2, 1);
    // Brain.Screen.print("dr45: %f", dr45.x);
    // Brain.Screen.setCursor(2, 20);
    // Brain.Screen.print(dr45.y);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("He:%.4f", He);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print(i);

    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("path.xy %f", _path[i].x);
    Brain.Screen.setCursor(5, 20);
    Brain.Screen.print(_path[i].y);

    L.spin(forward, lS, rpm);
    R.spin(forward, rS, rpm);
}

bool ppc::isNear(int i, double dist) {
    return Curve::distance(p.point(), _path[i]) < dist;
}

void ppc::run() {
    // Brain.Screen.setCursor(6, 1);
    // Brain.Screen.print("START run()");
    int i = 0;
    while (i < static_cast<int>(_path.size()) - 1) {
        // Brain.Screen.setCursor(6, 1);
        // Brain.Screen.print("i: %d", i);
        update();
        // Brain.Screen.setCursor(9, 1);
        // Brain.Screen.print(_path[i].x);
        // Brain.Screen.setCursor(9, 10);
        // Brain.Screen.print(_path[i].y);
        if (isNear(i, 1.0)) {
            // d::w([this, i]() {
            //     Controller1.Screen.print("isNear(%d)", i);
            //     Controller1.Screen.setCursor(2, 1);
            //     Controller1.Screen.print("%d,%d", _path[i].x, _path[i].y);
            // });
            i++;
            // Brain.Screen.setCursor(i, 1);
            // Brain.Screen.print(i);
            // wait(100, msec);
            continue;
        }
        i = lookahead(i);
        control(i);
        // if (isNear(i, _lookahead * 0.5))
        //     i = std::min(i + 1, static_cast<int>(_path.size()) - 1);
        wait(20, msec);
    }

    double _mmax = _max;
    _max *= 0.2;
    while (!isNear(_path.size() - 1)) {
        update();
        Brain.Screen.setCursor(7, 1);
        Brain.Screen.print(i);
        control(_path.size() - 1);
        wait(20, msec);
    }
    _max = _mmax;

    L.stop(brake);
    R.stop(brake);
}

// WARNING: Delete in release
void ppc::visualizePath() {
    Brain.Screen.clearScreen();

    // 绘制网格（辅助坐标系）
    Brain.Screen.setPenColor(white);
    for (int i = 0; i < 7; i++) {
        Brain.Screen.drawLine(5 + i * 30, 5, 5 + i * 30, 185); // 竖线
        Brain.Screen.drawLine(5, 5 + i * 30, 185, 5 + i * 30); // 横线
    }

    // 绘制场地边界（示例：蓝色竖线代表障碍或得分区）
    Brain.Screen.setPenColor(blue);
    Brain.Screen.drawLine(35, 65, 35, 125);   // 左边界
    Brain.Screen.drawLine(155, 65, 155, 125); // 右边界

    // 绘制机器人当前位置（红色圆点）
    Brain.Screen.setPenColor(red);
    int xx = 80, yy = 155; // 屏幕上的机器人初始显示位置（非真实坐标）
    Brain.Screen.drawCircle(xx, yy, 3);

    // 绘制贝塞尔路径（绿色）
    Brain.Screen.setPenColor(green);
    double scaling_factor = 0.5; // 缩放比例（将厘米转为屏幕像素）
    for (size_t i = 1; i < _path.size(); i++) {
        int x1 = xx - static_cast<int>(_path[i - 1].x * scaling_factor);
        int y1 = yy - static_cast<int>(_path[i - 1].y * scaling_factor);
        int x2 = xx - static_cast<int>(_path[i].x * scaling_factor);
        int y2 = yy - static_cast<int>(_path[i].y * scaling_factor);
        Brain.Screen.drawLine(x1, y1, x2, y2);
        wait(50, msec); // 慢速绘制便于观察
    }

    // 显示当前位姿信息
    Brain.Screen.setPenColor(white);
    Brain.Screen.setCursor(1, 30);
    Brain.Screen.print("x:%.2f cm", p.x);
    Brain.Screen.setCursor(2, 30);
    Brain.Screen.print("y:%.2f cm", p.y);
    Brain.Screen.setCursor(3, 30);
    Brain.Screen.print("theta:%.2f deg", p.theta * 180 / M_PI);
}
