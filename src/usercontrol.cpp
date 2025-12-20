#include <class/bezier.hpp>
#include <class/ppcontrol.hpp>
void pr1(Bezier p) {
    for (int i = 0; i < p.size(); i++) {
        break;
        Brain.Screen.setCursor(2 * i + 1, 1);
        Brain.Screen.print(p.getPath()[i].x);
        Brain.Screen.setCursor(2 * i + 2, 1);
        Brain.Screen.print(p.getPath()[i].y);
        wait(500, msec);
    }
}
void usercontrol() {
    L.setStopping(brake);
    R.setStopping(brake);

    // L.setStopping(coast);
    // R.setStopping(coast);

    odometer.set(1);
    wait(1000, msec);

    Optical.setLightPower(10, pct);
    while (1) {
        auto path4 = Bezier();
        path4.generate(0, 0, 0, 8, 0, 16, 0, 48, 48);
        auto route =
            ppc::Builder{}.path(path4).lookahead(2).kp(0.3).r(1).build();
        wait(3000, msec);
        route.setup();
        pr1(path4);
        // Brain.Screen.setCursor(10, 1);
        // Brain.Screen.print("AFTER setup()");
        // while (1) {
        //     route.update();
        //     wait(20, msec);
        // }
        route.run();
        return;
        float POWER_Left = 0;
        float POWER_Right = 0;

        // 获取摇杆纵向输入（Axis3，前进/后退），缩放为电压值（最大约12.8V）
        float go = Controller1.Axis3.position(pct) * 0.128;

        // 计算转向分量（r）：根据横向摇杆（Axis1）位置，模拟非线性转向响应
        float r = 0;
        // 使用循环累加方式实现“加速转向”效果（越偏转，转向越灵敏）
        for (int i = 0; i <= abs(Controller1.Axis1.position(pct)); i++) {
            if (Controller1.Axis1.position(pct) >= 0)
                r += i * 0.0016; // 向右转
            else
                r += i * (-0.0016); // 向左转
        }

        // 差速驱动模型：左轮 = 前进 + 转向，右轮 = 前进 - 转向
        POWER_Left = (go + r);
        POWER_Right = (go - r);

        // 限制电压输出在 [-12.8V, +12.8V]
        // 范围内（V5电机最大电压为12V，此处略超可能是容错）
        if (POWER_Left > 12.8)
            POWER_Left = 12.8;
        else if (POWER_Left < -12.8)
            POWER_Left = -12.8;
        if (POWER_Right > 12.8)
            POWER_Right = 12.8;
        else if (POWER_Right < -12.8)
            POWER_Right = -12.8;

        // 死区处理：若左右轮输出都很小（±3V以内），则完全停止电机，避免微抖动
        if (POWER_Left <= 3 && POWER_Left >= -3 && POWER_Right <= 3 &&
            POWER_Right >= -3) {
            L.stop();
            R.stop();
        } else {
            // 否则按计算电压驱动左右轮
            L.spin(forward, POWER_Left, volt);
            R.spin(forward, POWER_Right, volt);
        }

        // // ———————————————— 机构控制部分 ————————————————

        // // 【吸球机构 Intake 控制】
        // // L1：正转（吸入）
        // if (Controller1.ButtonL1.pressing() &&
        //     !Controller1.ButtonL2.pressing() &&
        //     (!Controller1.ButtonR1.pressing() ||
        //      !Controller1.ButtonR2.pressing())) {
        //     Export.stop(hold); // 停止出球机构并保持位置
        //     Intake.spin(forward, 100, vex::velocityUnits::pct);
        //     Intake2.spin(forward, 100, vex::velocityUnits::pct);
        // }
        // // L2：反转（吐出）
        // else if (Controller1.ButtonL2.pressing() &&
        //          !Controller1.ButtonL1.pressing()) {
        //     Intake.spin(reverse, 100, vex::velocityUnits::pct);
        //     Intake2.spin(reverse, 100, vex::velocityUnits::pct);
        // }
        // // L1 + L2 同时按：慢速反转（可能是微调或防卡）
        // else if (Controller1.ButtonL1.pressing() &&
        //          Controller1.ButtonL2.pressing()) {
        //     Intake.spin(reverse, 20, vex::velocityUnits::pct);
        //     Intake2.spin(reverse, 40, vex::velocityUnits::pct);
        // }
        // // 其他情况：停止吸球机构
        // else {
        //     Intake.stop();
        //     Intake2.stop();
        // }

        // // 【出球机构 Export 控制】
        // if (Controller1.ButtonR1.pressing()) {
        //     Export.spin(fwd, 100, vex::velocityUnits::pct); //
        //     R1：正转（发射）
        // } else if (Controller1.ButtonR2.pressing()) {
        //     Export.spin(
        //         reverse, 40,
        //         vex::velocityUnits::pct); // R2：反转（回收） lr80 skill40
        // } else {
        //     Export.stop(); // 松开则停止
        // }

        // // 【前挡板控制】
        // // front_panel_state
        // // 是一个全局布尔变量，表示挡板当前是否处于“可触发”状态
        // if (Controller1.ButtonA.pressing() && front_panel_state) {
        //     front_panel_state = 0; // 防止重复触发
        //     front_panel_control(); // 执行挡板动作（如伸出/收回）
        // }
        // // 注册 A 键释放回调，用于重置状态（允许下次触发）
        // Controller1.ButtonA.released(front_panel_released);

        // // 【双钩机构控制】
        // // Double_hook_state 类似，用于控制钩子动作
        // if (Controller1.ButtonY.pressing() && Double_hook_state) {
        //     Double_hook_state = 0;
        //     Double_hook_control(); // 执行钩子动作（如放下/抬起）
        // }
        // Controller1.ButtonY.released(Double_hook_released); // 释放时重置状态

        // // 【升降机构控制（气动或数字输出）】
        // // ButtonDown：激活升降（如抬起支架）
        // if (Controller1.ButtonDown.pressing()) {
        //     Lift_it_up.set(true);
        // }
        // // ButtonUp：关闭升降
        // else if (Controller1.ButtonUp.pressing()) {
        //     Lift_it_up.set(false);
        // }
        /*
            按键
            @  R2 导入桶挡板
            @  L1 收球
            @  L2 吐球
            @  L1&L2 中桥上
            @  R1 长桥挡板
            */

        Brain.Screen.setCursor(1, 1);
        if (Optical.color() == red)
            Brain.Screen.print("red");
        if (Optical.color() == blue)
            Brain.Screen.print("blue");
        if (Controller1.ButtonL1.pressing() &&
            !Controller1.ButtonL2.pressing()) {
            // 按了L1且没有按L2 ---->收球
            Intake.spin(forward, 100, vex::velocityUnits::pct);
            transmit.spin(forward, 100, vex::velocityUnits::pct);
            if (Optical.color() != red && Optical.color() != blue) {
                Export.spin(forward, 100, vex::velocityUnits::pct);
            } else if (!Controller1.ButtonR1.pressing()) {
                Export.stop();
            }
        } else if (Controller1.ButtonL1.pressing() &&
                   Controller1.ButtonL2.pressing()) {
            // 按了L1且按了L2 ----> 中上桥   (判色？？？)
            Intake.spin(forward, 100, vex::velocityUnits::pct);
            transmit.spin(forward, 100, vex::velocityUnits::pct);
            Export.spin(reverse, 50, vex::velocityUnits::pct);
        } else if (!Controller1.ButtonL1.pressing() &&
                   Controller1.ButtonL2.pressing()) {
            // 按了L2，没有按L1 --->中下桥
            transmit.spin(reverse, 100, vex::velocityUnits::pct);
            Intake.spin(reverse, 12, vex::velocityUnits::pct);
            Export.spin(reverse, 100, vex::velocityUnits::pct);
            IntakeCylinder.set(true);
        } else {
            Intake.stop();
            transmit.stop();
            Export.stop();
            IntakeCylinder.set(false);
        }

        // 导入桶挡板
        if (Controller1.ButtonR2.pressing())
            Import_bucket_baffle.set(true);
        else
            Import_bucket_baffle.set(false);

        // 挡板控制
        if (Controller1.ButtonR1.pressing()) {
            Long_bridge_baffle.set(true);
            Export.spin(forward, 100, vex::velocityUnits::pct);
        } else {
            Long_bridge_baffle.set(false);
        }
        wait(20, msec);
    }
}
