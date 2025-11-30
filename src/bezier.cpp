#include <vex.h>

class Bezier {
  private:
    std::vector<Point> path_;

  public:
    void setPath() { return; }
    std::vector<Point> getPath() { return path_; }
    Point calcSingle_(Point p0, Point p1, Point p2, Point p3, double t) {
        double t2 = t * t;
        double t3 = t2 * t;
        double mt = 1 - t;
        double mt2 = mt * mt;
        double mt3 = mt2 * mt;
        return mt3 * p0 + 3 * mt2 * t * p1 + 3 * mt * t2 * p2 + t3 * p3;
    }
    void generate(Point p0, Point p1, Point p2, Point p3, int num) {
        path_.clear();
        for (int i = 0; i < num; i++) {
            double t = (double)i / num - 1;
            path_.push_back(calcSingle_(p0, p1, p2, p3, t));
        }
    }
};
