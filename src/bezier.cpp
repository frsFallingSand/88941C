#pragma once
#include "curve.cpp"
#include <class/bezier.hpp>
#include <vex.h>

std::vector<Point> Bezier::getPath() const { return _path; }
Point Bezier::calcSingle(Point p0, Point p1, Point p2, Point p3,
                         double t) const {
    double t2 = t * t;
    double t3 = t2 * t;
    double mt = 1 - t;
    double mt2 = mt * mt;
    double mt3 = mt2 * mt;
    return mt3 * p0 + 3 * mt2 * t * p1 + 3 * mt * t2 * p2 + t3 * p3;
}
void Bezier::resample(double spL) { _path = Curve::resample(_path, spL); }
void Bezier::generate(Point p0, Point p1, Point p2, Point p3, int num) {
    _path.clear();
    for (int i = 0; i < num; i++) {
        double t = (double)i / (num - 1);
        _path.push_back(calcSingle(p0, p1, p2, p3, t));
    }
}
void Bezier::generate(double p0x, double p0y, double p1x, double p1y,
                      double p2x, double p2y, double p3x, double p3y, int num) {
    generate(Point(p0x, p0y), Point(p1x, p1y), Point(p2x, p2y), Point(p3x, p3y),
             num);
}

size_t Bezier::size() const { return _path.size(); }
