#pragma once
#include "vex.h"
#include <vector>

class Bezier {
  private:
    std::vector<Point> _path;

  public:
    std::vector<Point> getPath() const;
    Point calcSingle(Point p0, Point p1, Point p2, Point p3, double t) const;
    void resample(double spL);
    void generate(Point p0, Point p1, Point p2, Point p3, int num);
    void generate(double p0x, double p0y, double p1x, double p1y, double p2x,
                  double p2y, double p3x, double p3y, int num);
    size_t size() const;
};
