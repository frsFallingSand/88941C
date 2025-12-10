#include "bezier.cpp"
#include "ppcontrol.cpp"

void Rdebug() {
    auto p0 = Point(0, 0);
    auto p1 = Point(34, 43);
    auto p2 = Point(40, -21);
    auto p3 = Point(42, 29);

    auto path1 = Bezier();
    path1.generate(p0, p1, p2, p3, 100);

    auto route1 = ppc::Builder{}.path(path1).build();
    route1.setup();

    int i = 0;
    while (i < path1.size() - 1) {
        route1.update();
        route1.control(i);
        route1.visualizePath();
        i++;
        wait(20, msec);
    }

    // 到达终点，停车
    L.stop();
    R.stop();
}
