#pragma once
#include "bezier.cpp"
#include "curve.cpp"
#include <cmath>
#include <limits>
#include <vector>
// #include <vex.h>

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
    int _i;
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
        Builder &backward(bool a) {
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
        Builder &x(double x) {
            lastX = x;
            return *this;
        }
        Builder &y(double y) {
            lastY = y;
            return *this;
        }
        Builder &xy(Point a) {
            lastX = a.x;
            lastY = a.y;
            return *this;
        }
        Builder &i(int a) {
            _i = a;
            return *this;
        }
        ppc build() const {
            return ppc(_max, _min, _path, _backward, _lookahead, _kp, _r, _tpr,
                       _width, lastX, lastY, _i);
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
        double lastX = 0;
        double lastY = 0;
        int _i = 0;

        std::vector<Point> _path;
    };

    ppc(double max, double min, std::vector<Point> p, bool bw, double l,
        double kp, double r, double tpr, double width, double x, double y,
        int i)
        : _max(max), _min(min), _path(p), _backward(bw), _lookahead(l), _kp(kp),
          _r(r), _tpr(tpr), _width(width), lastX(x), lastY(y), _i(i) {}

    // ↑ MANUL BUILDER

    std::vector<Point> getPath() { return _path; }

    double normAngle(double a) {
        while (a > M_PI)
            a -= 2 * M_PI;
        while (a < -M_PI)
            a += 2 * M_PI;
        return a;
    }

    void setup() {
        // WARNING: Ensure you have init IMU

        p.x = lastX;
        p.y = lastY;

        x.resetPosition();
        y.resetPosition();
        lastX = x.position(degrees);
        lastY = y.position(degrees);

        p.theta = IMU.rotation(degrees) * M_PI / 180.0;
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
        p.theta = curT;

        lastX = curX;
        lastY = curY;

        //     Brain.Screen.setCursor(1, 1);
        //     Brain.Screen.print(p.x);
        //     Brain.Screen.setCursor(2, 1);
        //     Brain.Screen.print(p.y);
    }

    int lookahead(int startI) {
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

    void control(int i) {
        if (_path.empty())
            return;

        // i = lookahead(i);
        if (i == -1)
            return;

        Point t = _path[i];
        double tH = Curve::tangent(_path, i);
        if (_backward)
            tH = normAngle(tH + M_PI);

        Point d = t - p.point();
        double dx = d.x;
        double dy = d.y;

        Point dr = Point(dx * cos(-p.theta) - dy * sin(-p.theta),
                         dx * sin(-p.theta) + dy * cos(-p.theta));

        double ddis = Curve::distance(dr, Point(0, 0));

        double k = (ddis > 0.1) ? (2 * dr.x) / (ddis * ddis) : 0; // 曲率

        double He = normAngle(tH - p.theta);
        double Hc = He * _kp;

        double disT = Curve::distance(p.point(), t);
        double baseV = _min + (_max - _min) * std::min(1.0, disT / 50.0);
        if (_backward)
            baseV = -baseV;

        // TODO: reverse check
        double lS = baseV * (1 + k * _width / 2) + Hc;
        double rS = baseV * (1 - k * _width / 2) - Hc;

        lS = clamp(lS, -_max, _max);
        rS = clamp(rS, -_max, _max);

        // Brain.Screen.setCursor(1, 1);
        // Brain.Screen.print(lS);
        // Brain.Screen.setCursor(2, 1);
        // Brain.Screen.print(rS);
        // pos lv rv curT
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print(p.x);
        Brain.Screen.setCursor(1, 10);
        Brain.Screen.print(p.y);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print(lS);
        Brain.Screen.setCursor(2, 10);
        Brain.Screen.print(rS);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print(normAngle(IMU.heading()));
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print(i);

        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print(_path[i].x);
        Brain.Screen.setCursor(6, 1);
        Brain.Screen.print(_path[i].y);

        L.spin(forward, lS, rpm);
        R.spin(forward, rS, rpm);
    }

    bool isNear(int i, double dist = 0.5) {
        return Curve::distance(p.point(), _path[i]) < dist ? true : false;
    }

    void run() {
        Brain.Screen.setCursor(10, 1);
        Brain.Screen.print("START run()");
        int i = 0;
        while (i < static_cast<int>(_path.size()) - 1) {
            Brain.Screen.setCursor(10, 1);
            Brain.Screen.print("IN run()");
            update();
            if (isNear(i)) {
                i++;
                continue;
            }
            i = lookahead(i);
            control(i);
            Brain.Screen.setCursor(8, 1);
            Brain.Screen.print(i);
            wait(20, msec);
        }

        while (!isNear(_path.size() - 1)) {
            Brain.Screen.setCursor(8, 1);
            Brain.Screen.print(i);
            control(_path.size() - 1);
            wait(20, msec);
        }

        L.stop();
        R.stop();
    }

    // WARNING: Delete in release
    void visualizePath() {
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
};
