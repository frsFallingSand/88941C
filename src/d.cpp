#include <class/d.h>
#include <functional>
#include <vex.h>
namespace d {
void w(std::function<void()> f) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    f();
    wait(100, msec);
    while (!Controller1.ButtonA.pressing())
        wait(100, msec);
}
} // namespace d
