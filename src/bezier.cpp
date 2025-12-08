#include <vex.h>

class Bezier {
  private:
    std::vector<Point> _path;

  public:
    void setPath() { return; }
    std::vector<Point> getPath() { return _path; }
    Point calcSingle(Point p0, Point p1, Point p2, double t) {
        double t2 = t * t;
        double mt = 1 - t;
        double mt2 = mt * mt;
        return mt2 * p0 + 2 * t * mt * p1 + t2 * p2;
    }
    Point calcSingle(Point p0, Point p1, Point p2, Point p3, double t) {
        double t2 = t * t;
        double t3 = t2 * t;
        double mt = 1 - t;
        double mt2 = mt * mt;
        double mt3 = mt2 * mt;
        return mt3 * p0 + 3 * mt2 * t * p1 + 3 * mt * t2 * p2 + t3 * p3;
    }
    void generate(Point p0, Point p1, Point p2, Point p3, int num) {
        _path.clear();
        for (int i = 0; i < num; i++) {
            double t = (double)i / num - 1;
            _path.push_back(calcSingle(p0, p1, p2, p3, t));
        }
    }
    void generate(Point p0, Point p1, Point p2, int num) {
        _path.clear();
        for (int i = 0; i < num; i++) {
            double t = (double)i / num - 1;
            _path.push_back(calcSingle(p0, p1, p2, t));
        }
    }
};
