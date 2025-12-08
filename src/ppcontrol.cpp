#pragma once
#include "bezier.cpp"
#include "curve.cpp" // WARNING: Not the recommended solution
#include "vex_global.h"
#include <cmath>
#include <limits>
#include <vector>
// #include <vex.h>

Curve c = Curve();

class ppc {
  private:
    double _r;
    double _tpr;
    double _width;
    double _lookahead;
    double _max;
    double _min;
    double _kp; // use for heading correct
    bool _backward;
    std::vector<Point> _path;

    Pose p;
    double lastX = 0;
    double lastY = 0;

  public:
    // ↓ MANUL BUILDER

    class Builder {
      public:
        Builder &max(double v) {
            _max = v;
            return *this;
        }
        Builder &min(double v) {
            _min = v;
            return *this;
        }
        Builder &path(std::vector<Point> p) {
            _path = p;
            return *this;
        }
        Builder &path(Bezier p) {
            _path = p.getPath();
            return *this;
        }
        Builder &backward(double a) {
            _backward = a;
            return *this;
        }
        Builder &lookahead(double a) {
            _lookahead = a;
            return *this;
        }
        Builder &kp(double k) {
            _kp = k;
            return *this;
        }
        ppc build() const {
            return ppc(_max, _min, _path, _backward, _lookahead, _kp, _r, _tpr,
                       _width);
        }

      private:
        double _r = 2.5;
        double _tpr = 360.0;
        double _width = 18.5;
        double _lookahead = 28.5;
        double _max = 80.0;
        double _min = 25.0;
        double _kp = 3.0; // use for heading correct
        bool _backward = 0;

        std::vector<Point> _path;
    };

    ppc(double max, double min, std::vector<Point> p, bool bw, double l,
        double kp, double r, double tpr, double width)
        : _max(max), _min(min), _path(p), _backward(bw), _lookahead(l), _kp(kp),
          _r(r), _tpr(tpr), _width(width) {}

    // ↑ MANUL BUILDER

    double normAngle(double a) {
        while (a > M_PI)
            a -= 2 * M_PI;
        while (a < -M_PI)
            a += 2 * M_PI;
        return a;
    }

    void setup() {
        // WARNING: Ensure you have init IMU

        x.resetPosition();
        y.resetPosition();
        lastX = x.position(degrees);
        lastY = y.position(degrees);

        p.theta = IMU.rotation(degrees) * M_PI / 180.0;
        p.x = 0;
        p.y = 0;
    }

    void update() {
        double curX = x.position(degrees);
        double curY = y.position(degrees);

        double dX = ((curX - lastX) / _tpr) * (2 * M_PI * _r);
        double dY = ((curY - lastY) / _tpr) * (2 * M_PI * _r);

        double curT = IMU.rotation(degrees) * M_PI / 180.0;

        double gdX = dX * cos(p.theta) - dY * sin(p.theta);
        double gdY = dX * sin(p.theta) + dY * cos(p.theta);

        p.x += gdX;
        p.y += gdY;
        p.theta += curT;

        lastX = curX;
        lastY = curY;
    }

    int lookahead(const std::vector<Point> &path, int startI) {
        int closestI = startI;
        double minD = std::numeric_limits<double>::max();
        for (int i = startI; i < path.size(); i++) {
            double d = c.distance(p.point(), path[i]);
            if (d < minD) {
                minD = d;
                closestI = i;
            }
        }

        for (int i = closestI; i < path.size(); i++) {
            double d = c.distance(p.point(), path[i]);
            if (d >= _lookahead) {
                return i;
            }
        }

        return path.size() - 1;
    }

    void control(const std::vector<Point> &path, int i) {
        if (path.empty())
            return;

        i = lookahead(path, i);

        Point t = path[i];
        double tH = c.tangent(path, i);

        Point d = t - p.point();
        double dx = d.x;
        double dy = d.y;

        Point dr = Point(dx * cos(-p.theta) - dy * sin(-p.theta),
                         dx * sin(-p.theta) + dy * cos(-p.theta));

        double ddis = c.distance(dr, Point(0, 0));

        double k = (ddis > 0.1) ? (2 * dr.x) / (ddis * ddis) : 0; // 曲率

        double He = normAngle(tH - p.theta);
        double Hc = He * _kp;

        double disT = c.distance(p.point(), t);
        double baseV = _min + (_max - _min) * std::min(1.0, disT / 50.0);

        // TODO: reverse
        double lS = baseV * (1 + k * _width / 2) + Hc;
        double rS = baseV * (1 - k * _width / 2) - Hc;

        lS = clamp(lS, -_max, _max);
        rS = clamp(rS, -_max, _max);

        if (!_backward) {
            L.spin(forward, lS, rpm);
            R.spin(forward, rS, rpm);
        } else {
            L.spin(reverse, rS, rpm);
            R.spin(reverse, lS, rpm);
        }
    }

    // WARNING: Delete in release
    void visualizePath(const std::vector<Point> &path) {
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
        for (size_t i = 1; i < path.size(); i++) {
            int x1 = xx - static_cast<int>(path[i - 1].x * scaling_factor);
            int y1 = yy - static_cast<int>(path[i - 1].y * scaling_factor);
            int x2 = xx - static_cast<int>(path[i].x * scaling_factor);
            int y2 = yy - static_cast<int>(path[i].y * scaling_factor);
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
};
