#include "bezier.cpp"
#include "ppcontrol.cpp"

void Rdebug() {
    auto p0 = Point(0, 0);
    auto p1 = Point(34, 43);
    auto p2 = Point(40, -21);
    auto p3 = Point(42, 29);

    auto path1 = Bezier();
    path1.generate(p0, p1, p2, p3, 20);

    auto path2 = Bezier();
    path2.generate(0, 0, 0, 40, 35, 70, 75, 80, 20);

    auto path3 = Bezier();
    path3.generate(0, 0, 10, 10, 15, 20, 20);

    auto route = ppc::Builder{}.path(path3).build();
    route.setup();

    int i = 0;
    while (i < path1.size() - 1) {
        route.update();
        route.control(i);
        route.visualizePath();
        i++;
        wait(20, msec);
    }

    // 到达终点，停车
    L.stop();
    R.stop();
}
