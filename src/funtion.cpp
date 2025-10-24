#include "vex.h"

// 定义左右电机组（每侧3个电机）
motor_group L = motor_group(L1, L2, L3);  // 左侧电机组
motor_group R = motor_group(R1, R2, R3);  // 右侧电机组

// 全局参数定义
double kV = 0.3;            // 前馈动力系数（0.3–0.7），用于辅助大角度转向时提供基础动力
double brakePower = 0.3;    // 刹车力度（0.4–0.8），值越小，越接近目标角度时减速越快（此处设为0.3，可能偏激进）
double P = 0, I = 0, D = 0; // PID 各项输出缓存
double feedforward = 0;     // 前馈输出值

// ==============================================================================
// 智能转向函数：使用 PID + 前馈 + 刹车机制实现精准转向
// 参数：
//   targetAngle：目标角度（单位：度）
//   kP, kI, kD：PID 控制参数
// ==============================================================================
void smartTurn(double targetAngle, double kP, double kI, double kD) {
    double error = 0;           // 当前角度误差
    double prevError = 0;       // 上一次误差（用于 D 项计算）
    double integral = 0;        // 积分项（用于 I 项）
    float firsttime;            // 用于记录误差进入容差范围的起始时间
    float starttime = Brain.Timer.value();
    bool ck = 1;

    // 注：陀螺仪校准部分被注释掉了，实际使用时可能需要手动校准或确保启动前已校准
    // IMU.calibrate();
    // while(IMU.isCalibrating()) wait(10, msec);
    // IMU.resetHeading();
    // wait(300, msec);

    // 主控制循环
    while (true) {
        if (Brain.Timer.value() - starttime >= 3){
            L.stop();
            R.stop();
            break;
        }
        // 获取当前朝向（IMU.heading() 返回 0~360 度）
        double currentAngle = IMU.heading();
        error = targetAngle - currentAngle;

        // 将误差归一化到 [-180, 180] 范围内，避免跨 0°/360° 时的跳变
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        if (fabs(error) < 15 && ck){
            kP *= 0.7;
            ck = 0;
        }

        // ========== 前馈部分（当前被注释，可按需启用） ==========
        // 根据误差大小动态设定前馈值（大误差高速，小误差低速）
        /*
        feedforward = 0;
        if (fabs(error) > 50) {
            feedforward = kV * 100;  // 大角度全速转
        } else if (fabs(error) > 30) {
            feedforward = kV * 60;   // 中等角度中速
        } else {
            feedforward = kV * 10;   // 小角度慢速
        }
        */

        // ========== PID 控制部分 ==========
        // P 项：比例控制
        P = kP * error;

        // I 项：积分控制（带抗饱和和条件积分）
        if (fabs(error) < 15) {
            integral += error;  // 仅在误差较小时累积积分
        } else {
            integral = 0;       // 误差大时清零积分，防止超调
        }
        // 积分限幅（防止积分饱和）
        if (fabs(integral) > 3000)
            integral = 3000 * (integral > 0 ? 1 : -1);
        I = kI * integral;

        // D 项：微分控制（抑制振荡）
        D = kD * (error - prevError);

        // 总输出 = PID + 前馈
        double output = P + I + D;

        // 添加前馈（根据转向方向调整符号）
        if (error > 0) {
            output += feedforward;  // 需要右转（顺时针）
        } else {
            output -= feedforward;  // 需要左转（逆时针）
        }

        // 刹车逻辑：当接近目标（误差 < 20°）时降低输出功率
        if (fabs(error) < 20) {
            output *= brakePower;
        }

        // 输出限幅（电机功率范围：-100% ~ +100%）
        if (output > 100) output = 100;
        if (output < -100) output = -100;

        // 驱动电机：差速转向（左轮正转，右轮反转实现原地转向）
        L.spin(fwd, output, pct);
        R.spin(fwd, -output, pct);  // 注意右轮方向相反

        // 判断是否稳定到达目标（误差 < 1° 且持续 50ms）
        if (fabs(error) < 1.0) {
            if (Brain.Timer.value() - firsttime >= 0.05) {  // 0.05秒 = 50ms
                L.stop();
                R.stop();
                break;  // 退出循环
            }
        } else {
            firsttime = Brain.Timer.value();  // 重置计时起点
        }

        // 更新前次误差
        prevError = error;

        // 控制循环周期：20ms
        wait(20, msec);
    }

    // 最终停止电机（保险）
    L.stop();
    R.stop();
}

// ==============================================================================
// 带 IMU 修正的平滑直线移动函数（含减速段）
// 参数：
//   dir：移动方向（fwd / reverse）
//   dis：总移动距离（以电机编码器度数为单位）
//   Maxspeed：初始最大速度（%）
//   DecelerationDis：开始减速的距离阈值（度）
//   Minspeed：最低速度（%）
//   targetdeg：期望保持的朝向角度（用于 IMU 纠偏）
//   kp：纠偏的比例系数（负值表示反向修正）
// ==============================================================================
void linearSmoothStop(vex::directionType dir, float dis, float Maxspeed,
                      float DecelerationDis, float Minspeed,
                      int targetdeg, double kp) {
    MoveDistancePID(dir, dis, Maxspeed, Minspeed, targetdeg, 0.2, 0.5);
    return;
    // 重置左右电机编码器
    L.setPosition(0, degrees);
    R.setPosition(0, degrees);
    wait(40, msec);  // 等待编码器稳定

    while (1) {
        // 计算当前朝向误差（用于横向纠偏）
        int deg_error = targetdeg - IMU.heading();

        // 若平均移动距离已达目标，停止
        if ((fabs(L.position(deg)) + fabs(R.position(deg))) / 2 >= dis) {
            L.stop(brake);
            R.stop(brake);
            break;
        }

        // 进入减速区：逐步降低速度（指数衰减）
        if ((fabs(L.position(deg)) + fabs(R.position(deg))) / 2 >= DecelerationDis) {
            Maxspeed *= 0.98;  // 每次循环降低 2%
        }

        // 保证速度不低于设定最小值
        Maxspeed = fmax(Maxspeed, Minspeed);

        // 计算纠偏量（差速控制：左快右慢 或 右快左慢）
        int k = deg_error * kp;  // kp 通常为负（如 -0.5）

        // 应用纠偏后的速度
        L.spin(dir, Maxspeed + k, vex::velocityUnits::pct);
        R.spin(dir, Maxspeed - k, vex::velocityUnits::pct);

        wait(10, msec);
    }
}

// ==============================================================================
// 基础移动函数（无 IMU 纠偏，仅依赖编码器）
// 参数：
//   dir：方向
//   dis：距离（电机转动度数）
//   v：速度（%）
// ==============================================================================
void move(vex::directionType dir, float dis, int v) {
    // 同步运行左右电机（非阻塞左轮，阻塞右轮以同步结束）
    L.spinFor(dir, dis, deg, v, vex::velocityUnits::pct, false);
    R.spinFor(dir, dis, deg, v, vex::velocityUnits::pct);
    // 到达后刹车
    L.stop(brake);
    R.stop(brake);
}

// ==============================================================================
// IMU 初始化与调试显示函数
// 在 IMU 校准期间持续显示当前朝向（用于手动确认校准完成）
// ==============================================================================
void init() {
    Brain.Screen.clearScreen();
    // 循环直到 IMU 开始校准（实际逻辑可能有误：应为 while(IMU.isCalibrating())）
    // 此处可能是为了等待用户手动触发校准？
    while (!IMU.isCalibrating()) {
        Controller1.Screen.clearScreen();
        Brain.Screen.clearScreen();

        Brain.Screen.setCursor(5, 22);
        Controller1.Screen.setCursor(2, 11);
        Controller1.Screen.print(IMU.heading(degrees));  // 控制器屏幕显示
        Brain.Screen.print(IMU.heading(degrees));        // 主脑屏幕显示

        // 调试提示：显示左右边界角度（338° 和 22°，可能对应场地特定方向）
        Brain.Screen.setCursor(5, 11);
        Brain.Screen.print("Left:338-%f", IMU.heading(degrees));
        Brain.Screen.setCursor(8, 11);
        Brain.Screen.print("Right:22-%f", IMU.heading(degrees));

        wait(50, msec);
    }
}

// ==============================================================================
// 按时间移动函数（适用于无编码器反馈的粗略移动）
// 参数：
//   dir：方向
//   sp：速度（%）
//   times：持续时间（毫秒）
// ==============================================================================
void moveTime(vex::directionType dir, double sp, double times) {
    L.spin(dir, sp, vex::velocityUnits::pct);
    R.spin(dir, sp, vex::velocityUnits::pct);
    wait(times, msec);
    L.stop(brake);
    R.stop(brake);
}

// ==============================================================================
// 前挡板控制（单次按键切换状态）
// 使用状态标志防止按键持续触发
// ==============================================================================
bool front_panel_state = 1;   // 1=可触发，0=已触发（防抖）
int front_panel_cnt = 0;      // 切换计数器（偶数开，奇数关）

void front_panel_control() {
    if (front_panel_state == 1) {  // 仅在松开后首次按下时执行
        front_panel_state = 0;     // 锁定状态
        if (front_panel_cnt % 2 == 0) {
            front_panel.set(true);   // 打开挡板
        } else {
            front_panel.set(false);  // 关闭挡板
        }
        front_panel_cnt++;
    }
}

// 按键松开时重置状态，允许下次触发
void front_panel_released() {
    front_panel_state = 1;
}

// ==============================================================================
// 双边钩子控制（逻辑同前挡板）
// ==============================================================================
bool Double_hook_state = 1;
int Double_hook_cnt = 0;

void Double_hook_control() {
    if (Double_hook_state == 1) {
        Double_hook_state = 0;
        if (Double_hook_cnt % 2 == 0) {
            Double_hook.set(true);
        } else {
            Double_hook.set(false);
        }
        Double_hook_cnt++;
    }
}

void Double_hook_released() {
    Double_hook_state = 1;
}

// ==============================================================================
// 自动化任务：从得分区运球到桥区（Push Back 比赛策略）
// 流程：
//   1. 打开前挡板
//   2. 启动滚筒进球
//   3. 向前冲一段
//   4. 后退微调
//   5. 后退至桥区（带 IMU 纠偏）
//   6. 抬升机构 + 反向吐球
//   7. 复位
// ==============================================================================
void Bucket_to_Bridge() {
    front_panel.set(true);                     // 打开前挡板
    wait(100, msec);
    
    Intake.spin(forward, 100, vex::velocityUnits::pct);   // 启动左右滚筒
    Intake2.spin(forward, 100, vex::velocityUnits::pct);
    
    moveTime(fwd, 30, 900);                    // 向前冲 900ms
    move(reverse, 5, 25);                      // 微退 5 度
    wait(1, sec);                              // 等待稳定

    // 后退至桥区（约 1200 度，保持朝向 180°）
    linearSmoothStop(reverse, 1200, 50, 800, 50, 180, -0.5);
    
    moveTime(reverse, 80, 400);                // 快速后退冲上桥
    wait(100, msec);

    // 启动抬升机构并反向吐球
    Export.spin(fwd, 100, vex::velocityUnits::pct);
    Intake.spin(reverse, 100, vex::velocityUnits::pct);
    Intake2.spin(reverse, 100, vex::velocityUnits::pct);
    wait(500, msec);

    // 停止吐球，再正转吸住残余球（防掉落）
    Intake.stop();
    Intake2.stop();
    Intake.spin(forward, 100, vex::velocityUnits::pct);
    Intake2.spin(forward, 100, vex::velocityUnits::pct);
    wait(2, sec);

    // 停止所有机构
    Intake.stop();
    Intake2.stop();
    Export.stop();
    front_panel.set(false);                    // 关闭前挡板

    move(fwd, 300, 20);                        // 前进脱离桥区
    moveTime(reverse, 100, 300);               // 快速后退复位
}

// ==============================================================================
// 简化版 PID 转向函数（使用电压控制而非百分比）
// 参数：
//   t：目标角度
//   kp, ki, kd：PID 参数（注意 kd 未使用！）
//   minVolt：最低有效电压（防止电机堵转）
// ==============================================================================
void pid(double t, double kp, double ki, double kd, double minVolt) {
    double firsttime = 0;
    double sumi = 0;  // 积分累计

    while (1) {
        double error = t - IMU.heading();

        // 归一化误差到 [-180, 180]
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        // P 项
        double P = error * kp;

        // I 项（条件积分 + 限幅）
        if (fabs(error) < 10) {
            sumi += error;
        } else {
            sumi = 0;
        }
        if (fabs(sumi) > 700)
            sumi = 700 * (sumi > 0 ? 1 : -1);
        double I = sumi * ki;

        // 注意：D 项未计算！函数名虽叫 pid，但实际是 PI 控制
        double PID = P + I;

        // 到达目标判断（误差 <1° 且持续 50ms）
        if (fabs(error) < 1.0) {
            if (Brain.Timer.value() - firsttime >= 0.05) {
                L.stop();
                R.stop();
                break;
            }
        } else {
            firsttime = Brain.Timer.value();
        }

        // 电压输出限幅（VEX V5 电机电压范围约 ±12.8V）
        if (PID >= 12.8) PID = 12.8;
        if (PID <= -12.8) PID = -12.8;

        // 设置死区：小输出时用最小电压驱动（克服静摩擦）
        if (PID > 0 && PID <= 2) PID = minVolt;
        if (PID < 0 && PID >= -2) PID = -minVolt;

        // 电压模式驱动（更线性，适合 PID）
        L.spin(fwd, PID, volt);
        R.spin(fwd, -PID, volt);

        wait(20, msec);
    }
}

/*依托行程轮里程函数
@targetDist  单位为inch

*/
float accelerationRate = 0.4;//加速度
double lastYEncoder1;
void MoveDistancePID(vex:: directionType dir,float targetDist,float Maxspeed,float minSpeed,float targetAngle,float DISkp,float Anglekp){
  Controller1.Screen.clearScreen();
  float currentDist=0;
  lastYEncoder1=0;
  y.resetPosition();
  wait(100,msec);
  float remainingDist = targetDist - currentDist; //总剩余距离
  float currentSpeed;//启动速度
  if(remainingDist>=10&&Maxspeed>=20) currentSpeed = 20; //确定启动速度
  else currentSpeed = Maxspeed;  //最大速度小于启动速度的时候，按Maxspeed
  while(1){
    int deg_error = targetAngle - IMU.heading();
    // 确保误差在-180到180范围内
    if(deg_error > 180) deg_error -= 360;
    if(deg_error < -180) deg_error += 360;
    double currentYEncoder = y.position(degrees);
    double deltaY = ((currentYEncoder - lastYEncoder1) / 360) * (2 * M_PI * 1);
    currentDist+=deltaY;
    if(targetDist<=fabs(currentDist)){
      break;
    }
    //加速阶段
    if(currentSpeed<Maxspeed) currentSpeed+=accelerationRate; //加速
    remainingDist = targetDist - fabs(currentDist);
    currentSpeed = pow(remainingDist*DISkp,2)*Maxspeed; 
    
    int k = deg_error*Anglekp;

    currentSpeed = fmin(currentSpeed,Maxspeed);
    currentSpeed = fmax(currentSpeed,minSpeed);
    L.spin(dir,currentSpeed+k,pct);
    R.spin(dir,currentSpeed-k,pct);
    //wait(5,msec);
    lastYEncoder1 = currentYEncoder;
    Brain.Screen.setCursor(5,22);
    Brain.Screen.print(currentDist);
  }
  
}

#include <cmath>
#include <algorithm>

/**
 * @brief 根据当前已行驶路程，计算平滑的目标速度
 *
 * @param current_distance  当前已行驶的路程（由里程计获取）
 * @param total_distance    总目标路程
 * @param cruise_speed      匀速阶段的目标速度（> 0）
 * @param max_decel         最大允许减速度（正值，用于计算减速距离）
 * @return double           平滑后的目标速度
 */
double getSmoothTargetVelocity(
    double current_distance,
    double total_distance,
    double cruise_speed,
    double max_decel)
{
    // 安全检查
    if (cruise_speed <= 0.0 || max_decel <= 0.0 || total_distance <= 0.0) {
        return 0.0;
    }

    // 理论所需减速距离（从 cruise_speed 减速到 0，匀减速）
    double decel_distance = (cruise_speed * cruise_speed) / (2.0 * max_decel);

    // 可选：增加一点安全余量（例如 10%），防止因计算误差刹不住
    decel_distance *= 1.1;

    // 减速起始位置（不能小于0）
    double decel_start = std::max(0.0, total_distance - decel_distance);

    // 如果已经到达或超过总路程，停止
    if (current_distance >= total_distance) {
        return 0.0;
    }

    // 还未进入减速段：保持匀速
    if (current_distance < decel_start) {
        return cruise_speed;
    }

    // 进入平滑减速段：使用余弦函数平滑减速到 0
    double progress = (current_distance - decel_start) / (total_distance - decel_start);
    progress = clamp(progress, 0.0, 1.0); // 确保在 [0, 1] 范围内

    // 余弦平滑：v = v_cruise * (1 + cos(π * t)) / 2
    // t=0 → v = v_cruise, t=1 → v = 0，且加速度在两端为0
    double velocity = cruise_speed * (1.0 + std::cos(M_PI * progress)) / 2.0;

    // 确保速度非负
    return std::max(0.0, velocity);
}
