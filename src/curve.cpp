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

    std::vector<Point> resample(const std::vector<Point> &curve, double spL) {
        std::vector<Point> sp;
        double l = length(curve);
        int pNum = static_cast<int>(l / spL) + 1;
        if (pNum <= 1)
            return curve;
        sp.push_back(curve[0]);
        double sumL = 0;
    }
};
