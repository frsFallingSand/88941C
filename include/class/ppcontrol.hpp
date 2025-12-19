#pragma once

#include <cmath>
#include <vector>
#include <vex.h>

class ppc {
  public:
    // Builder 嵌套类（仅声明 public 接口）
    class Builder {
      public:
        Builder &max(double v);
        Builder &min(double v);
        Builder &path(const std::vector<Point> &p);
        Builder &path(const class Bezier &p); // 注意：这里用 class Bezier 前置
        Builder &backward(bool a);
        Builder &lookahead(double a);
        Builder &kp(double k);
        Builder &i(int a);
        Builder &r(double a);
        ppc build() const;

      private:
        double _r = 1.0;
        double _tpr = 360.0;
        double _width = 18.5;
        double _lookahead = 28.5;
        double _max = 80.0;
        double _min = 25.0;
        double _kp = 3.0;
        bool _backward = false;
        int _i = 0;
        std::vector<Point> _path;
    };

    // 构造函数
    ppc(double max, double min, const std::vector<Point> &p, bool bw, double l,
        double kp, double r, double tpr, double width, int i);

    // 公有方法（仅声明）
    std::vector<Point> getPath();
    double normAngle(double a);
    void setup();
    void update();
    int lookahead(int startI);
    void control(int i);
    bool isNear(int i, double dist = 0.5);
    void run();
    void visualizePath();

  private:
    double _r;
    double _tpr;
    double _width;
    double _lookahead;
    double _max;
    double _min;
    double _kp;
    bool _backward;
    int _i;
    std::vector<Point> _path;
    Pose p;
    double lastX, lastY;
};
