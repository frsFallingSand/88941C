#include "bezier.cpp"
#include "ppcontrol.cpp"

void Rdebug() {
    auto p0 = Point(0, 0);
    auto p1 = Point(34, 43);
    auto p2 = Point(40, -21);
    auto p3 = Point(42, 29);

    auto path1 = Bezier();
    path1.generate(p0, p1, p2, p3, 20);
    // path1.resample(1);

    // auto path2 = Bezier();
    // path2.generate(0, 0, 0, 40, 35, 70, 75, 80, 20);

    // auto path3 = Bezier();
    // path3.generate(0, 0, 10, 10, 15, 20, 20);

    auto path4 = Bezier();
    path4.generate(0, 0, 0, 10, 0, 20, 0, 30, 50);

    auto route = ppc::Builder{}.path(path4).build();
    route.setup();

    for (int i = 0; i < path1.size(); i++) {
        break;
        Brain.Screen.setCursor(2 * i + 1, 1);
        Brain.Screen.print(path1.getPath()[i].x);
        Brain.Screen.setCursor(2 * i + 2, 1);
        Brain.Screen.print(path1.getPath()[i].y);
        wait(500, msec);
    }

    // L.setStopping(coast);
    // R.setStopping(coast);

    route.run();

    L.stop();
    R.stop();
}
