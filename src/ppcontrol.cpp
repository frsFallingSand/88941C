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

    Pose p;
    double lastX = 0;
    double lastY = 0;
    std::vector<Point> _path;

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
            return ppc(_max, _min, _path, _backward, _lookahead, _kp);
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
        double kp)
        : _max(max), _min(min), _path(p), _backward(bw), _lookahead(l),
          _kp(kp) {}

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

    void pp(const std::vector<Point> &path, int i) {
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

        L.spin(forward, lS, rpm);
        R.spin(forward, rS, rpm);
    }
};
