#include <vector>
#include <vex.h>

class Curve {
  public:
    double distance(Point p1, Point p2) {
        Point p = p1 - p2;
        return sqrt(pow(p.x, 2) + pow(p.y, 2));
    }

    double length(const std::vector<Point> &curve) {
        double l = 0;
        for (size_t i = 1; i < curve.size(); i++) {
            l += distance(curve[i - 1], curve[i]);
        }
        return l;
    }
    Point linearInterpolation(Point p1, Point p2, double k) {
        return (1 - k) * p1 + k * p2;
    }
    std::vector<Point> resample(const std::vector<Point> &curve, double spL) {
        std::vector<Point> sp;
        double l = length(curve);
        int pNum = static_cast<int>(l / spL) + 1;
        if (pNum <= 1)
            return curve;
        sp.push_back(curve[0]);
        double sumL = 0;

        for (int i = 1; i < pNum; i++) {
            double expL = i * spL;
            for (int j = 1; j < curve.size(); j++) {
                double segL = distance(curve[j - 1], curve[j]);
                if (sumL + segL >= expL) {
                    double k = (expL - sumL) / segL;
                    sp.push_back(
                        linearInterpolation(curve[j - 1], curve[j], k));
                    sumL = expL;
                    break;
                }
                sumL += segL;
            }
        }

        if (distance(sp.back(), curve.back()) > 0.1) {
            sp.push_back(curve.back());
        }

        return sp;
    }
};
