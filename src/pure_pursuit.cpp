#include "vex.h"

// ========================
// 常量配置（根据机器人物理参数调整）
// ========================
const double WHEEL_RADIUS = 2.5; // 里程计轮子半径
const double ENCODER_TPR =
    360.0; // 编码器每转一圈的脉冲数（单位：度，V5电机默认360度/圈）
const double TRACK_WIDTH = 18.5;        // 左右轮之间的距离，用于差速转弯计算
const double LOOKAHEAD_DISTANCE = 28.5; // Pure Pursuit 算法的前瞻距离
const double MAX_SPEED = 80.0;          // 电机最大转速（单位：RPM）
const double MIN_SPEED = 25.0;          // 电机最小转速（防止低速卡顿）
const double Kp_heading = 3.0; // 朝向误差的比例系数（用于修正机器人朝向）

// ========================
// 全局变量
// ========================
Pose currentPose;              // 当前机器人在全局坐标系中的位姿（x, y, theta）
double lastXEncoder = 0;       // 上一次X轴编码器读数（用于计算位移增量）
double lastYEncoder1 = 0;      // 上一次Y轴编码器读数
std::vector<Point> bezierPath; // 存储生成的贝塞尔曲线路径点

// ========================
// 贝塞尔曲线相关函数
// ========================

// 计算三阶贝塞尔曲线上参数 t 对应的点（t ∈ [0, 1]）
Point bezierPoint(Point p0, Point p1, Point p2, Point p3, double t) {
    double t2 = t * t;
    double t3 = t2 * t;
    double mt = 1 - t;
    double mt2 = mt * mt;
    double mt3 = mt2 * mt;

    Point res;
    // 三阶贝塞尔公式：B(t) = (1−t)³P₀ + 3(1−t)²tP₁ + 3(1−t)t²P₂ + t³P₃
    res.x = mt3 * p0.x + 3 * mt2 * t * p1.x + 3 * mt * t2 * p2.x + t3 * p3.x;
    res.y = mt3 * p0.y + 3 * mt2 * t * p1.y + 3 * mt * t2 * p2.y + t3 * p3.y;
    return res;
}

// 生成贝塞尔路径：从 p0 到 p3，控制点为 p1、p2，共生成 num 个点
std::vector<Point> generateBezierPath(Point p0, Point p1, Point p2, Point p3,
                                      int num) {
    bezierPath.clear(); // 防止重复调用时路径叠加
    for (int i = 0; i < num; i++) {
        double t = (double)i / (num - 1); // t 从 0 到 1 均匀分布
        bezierPath.push_back(bezierPoint(p0, p1, p2, p3, t));
    }
    return bezierPath;
}

// ========================
// 初始化函数
// ========================

// 初始化 IMU（陀螺仪）和正交编码器（X/Y 轴）
void setup() {
    // 校准 IMU（必须在静止状态下进行）
    IMU.calibrate();
    while (IMU.isCalibrating()) {
        wait(50, msec);
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("IMU is Calibrating ");
    }

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("IMU is setup success ");
    wait(100, msec);

    // 重置编码器位置
    x.resetPosition();
    y.resetPosition();
    lastXEncoder = x.position(degrees);
    lastYEncoder1 = y.position(degrees);

    // 初始化机器人朝向（将 IMU 的度数转为弧度）
    currentPose.theta = IMU.rotation(degrees) * M_PI / 180.0;
    // 初始位置设为 (0, 0)
    currentPose.x = 0;
    currentPose.y = 0;
}

// ========================
// 里程计更新（融合编码器 + IMU）
// ========================

// 更新机器人当前位姿（x, y, theta）
void updateOdometry() {
    // 读取当前编码器值
    double currentXEncoder = x.position(degrees);
    double currentYEncoder = y.position(degrees);

    // 计算编码器增量（转为实际位移，单位：厘米）
    double deltaX = ((currentXEncoder - lastXEncoder) / ENCODER_TPR) *
                    (2 * M_PI * WHEEL_RADIUS);
    double deltaY = ((currentYEncoder - lastYEncoder1) / ENCODER_TPR) *
                    (2 * M_PI * WHEEL_RADIUS);

    // 获取当前朝向（IMU，单位：弧度）
    double currentTheta = IMU.rotation(degrees) * M_PI / 180.0;

    // 将机器人坐标系下的位移转换到全局坐标系（旋转矩阵）
    double globalDeltaX =
        deltaX * cos(currentPose.theta) - deltaY * sin(currentPose.theta);
    double globalDeltaY =
        deltaX * sin(currentPose.theta) + deltaY * cos(currentPose.theta);

    // 更新全局位置
    currentPose.x += globalDeltaX;
    currentPose.y += globalDeltaY;
    currentPose.theta = currentTheta; // 直接使用 IMU 数据（更准确）

    // 更新上一次编码器值
    lastXEncoder = currentXEncoder;
    lastYEncoder1 = currentYEncoder;
}

// ========================
// 工具函数
// ========================

// 计算两点间欧氏距离（传入坐标）
double distances(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// 重载：计算两个 Point 结构体之间的距离
double dis(const Point &p1, const Point &p2) {
    return distances(p1.x, p1.y, p2.x, p2.y);
}

// 将角度归一化到 [-π, π] 范围内（避免角度跳变）
double normalizeAngle(double angle) {
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}

// 计算整条路径的总长度（用于重采样）
double calculateCurveLength(const std::vector<Point> &curve) {
    double length = 0;
    for (size_t i = 1; i < curve.size(); i++) {
        length += dis(curve[i - 1], curve[i]);
    }
    return length;
}

// 按固定间距重新采样路径点（使路径点分布更均匀，利于 Pure Pursuit）
std::vector<Point> resampleCurve(const std::vector<Point> &curve,
                                 double pointSpacing) {
    std::vector<Point> resampled;
    double totalLength = calculateCurveLength(curve);
    int numPoints = static_cast<int>(totalLength / pointSpacing) + 1;
    if (numPoints <= 1)
        return curve;

    resampled.push_back(curve[0]); // 起点必须保留
    double accumulatedLength = 0;

    for (int i = 1; i < numPoints; i++) {
        double targetLength = i * pointSpacing;
        // 遍历原始路径，找到目标长度对应的位置
        for (size_t j = 1; j < curve.size(); j++) {
            double segmentLength = dis(curve[j - 1], curve[j]);
            if (accumulatedLength + segmentLength >= targetLength) {
                double ratio =
                    (targetLength - accumulatedLength) / segmentLength;
                Point newPoint;
                newPoint.x =
                    curve[j - 1].x + ratio * (curve[j].x - curve[j - 1].x);
                newPoint.y =
                    curve[j - 1].y + ratio * (curve[j].y - curve[j - 1].y);
                resampled.push_back(newPoint);
                accumulatedLength = targetLength;
                break;
            }
            accumulatedLength += segmentLength;
        }
    }

    // 确保终点也被包含
    if (dis(resampled.back(), curve.back()) > 0.1) {
        resampled.push_back(curve.back());
    }
    return resampled;
}

// 计算路径上某点的切线方向（作为期望朝向）
double calculateTangentAtPoint(const std::vector<Point> &curve, int index) {
    if (curve.size() < 2)
        return 0;

    if (index == 0) {
        // 起点：用前两个点计算方向
        return atan2(curve[1].y - curve[0].y, curve[1].x - curve[0].x);
    } else if (index == curve.size() - 1) {
        // 终点：用最后两个点计算方向
        return atan2(curve.back().y - curve[curve.size() - 2].y,
                     curve.back().x - curve[curve.size() - 2].x);
    } else {
        // 中间点：用前后点计算中心差分方向（更平滑）
        double dx = curve[index + 1].x - curve[index - 1].x;
        double dy = curve[index + 1].y - curve[index - 1].y;
        return atan2(dy, dx);
    }
}

// 在路径中查找满足前瞻距离的点（Pure Pursuit 核心）
int findLookaheadPoint(const std::vector<Point> &path, int startIndex) {
    // 第一步：找离机器人最近的路径点（避免回溯）
    int closestIndex = startIndex;
    double minDist = std::numeric_limits<double>::max();
    for (int i = startIndex; i < path.size(); i++) {
        double dist =
            distances(currentPose.x, currentPose.y, path[i].x, path[i].y);
        if (dist < minDist) {
            minDist = dist;
            closestIndex = i;
        }
    }

    // 第二步：从最近点开始向前找，第一个距离 ≥ LOOKAHEAD_DISTANCE 的点
    for (int i = closestIndex; i < path.size(); i++) {
        double dist =
            distances(currentPose.x, currentPose.y, path[i].x, path[i].y);
        if (dist >= LOOKAHEAD_DISTANCE) {
            return i;
        }
    }

    // 如果找不到，就用终点
    return path.size() - 1;
}

// 限制值在 [min, max] 范围内（C++17 之前没有 std::clamp）
/* double clamp(double temp, double min, double max) {
    if (temp > max)
        temp = max;
    if (temp < min)
        temp = min;
    return temp;
}
 */
// ========================
// Pure Pursuit 控制主函数（结合贝塞尔路径）
// ========================

void bezierPurePursuit(const std::vector<Point> &path,
                       int &currentTargetIndex) {
    if (path.empty())
        return;

    // 找到当前应追踪的前瞻点
    currentTargetIndex = findLookaheadPoint(path, currentTargetIndex);
    Point targetPoint = path[currentTargetIndex];

    // 获取该点的期望朝向（路径切线方向）
    double targetHeading = calculateTangentAtPoint(path, currentTargetIndex);

    // 将目标点转换到机器人坐标系下（便于计算横向误差）
    double dx = targetPoint.x - currentPose.x;
    double dy = targetPoint.y - currentPose.y;
    double robotDx =
        dx * cos(-currentPose.theta) - dy * sin(-currentPose.theta);
    double robotDy =
        dx * sin(-currentPose.theta) + dy * cos(-currentPose.theta);

    // 计算机器人到目标点的距离
    double Ldis = sqrt(robotDx * robotDx + robotDy * robotDy);

    // Pure Pursuit 曲率公式：κ = 2 * y / L²（y 是横向误差）
    double curvature = (Ldis > 0.1) ? (2 * robotDy) / (Ldis * Ldis) : 0;

    // 朝向误差修正（比例控制）
    double headingError = normalizeAngle(targetHeading - currentPose.theta);
    double headingCorrection = Kp_heading * headingError;

    // 自适应速度：离目标越近，速度越慢（但不低于 MIN_SPEED）
    double distToTarget =
        distances(currentPose.x, currentPose.y, targetPoint.x, targetPoint.y);
    double baseSpeed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) *
                                       std::min(1.0, distToTarget / 50.0);

    // 差速驱动速度分配（结合曲率和朝向修正）
    double leftSpeed =
        baseSpeed * (1 + curvature * TRACK_WIDTH / 2) + headingCorrection;
    double rightSpeed =
        baseSpeed * (1 - curvature * TRACK_WIDTH / 2) - headingCorrection;

    // 限制电机速度范围
    leftSpeed = clamp(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = clamp(rightSpeed, -MAX_SPEED, MAX_SPEED);

    // 控制左右电机
    L.spin(forward, leftSpeed, rpm);
    R.spin(forward, rightSpeed, rpm);
}

// ========================
// 路径可视化（在 V5 Brain 屏幕上绘制）
// ========================

void visualizePath(const std::vector<Point> &path) {
    Brain.Screen.clearScreen();

    // 绘制网格（辅助坐标系）
    Brain.Screen.setPenColor(white);
    for (int i = 0; i < 7; i++) {
        Brain.Screen.drawLine(5 + i * 30, 5, 5 + i * 30, 185); // 竖线
        Brain.Screen.drawLine(5, 5 + i * 30, 185, 5 + i * 30); // 横线
    }

    // 绘制场地边界（示例：蓝色竖线代表障碍或得分区）
    Brain.Screen.setPenColor(blue);
    Brain.Screen.drawLine(35, 65, 35, 125);   // 左边界
    Brain.Screen.drawLine(155, 65, 155, 125); // 右边界

    // 绘制机器人当前位置（红色圆点）
    Brain.Screen.setPenColor(red);
    int xx = 80, yy = 155; // 屏幕上的机器人初始显示位置（非真实坐标）
    Brain.Screen.drawCircle(xx, yy, 3);

    // 绘制贝塞尔路径（绿色）
    Brain.Screen.setPenColor(green);
    double scaling_factor = 0.5; // 缩放比例（将厘米转为屏幕像素）
    for (size_t i = 1; i < path.size(); i++) {
        int x1 = xx - static_cast<int>(path[i - 1].x * scaling_factor);
        int y1 = yy - static_cast<int>(path[i - 1].y * scaling_factor);
        int x2 = xx - static_cast<int>(path[i].x * scaling_factor);
        int y2 = yy - static_cast<int>(path[i].y * scaling_factor);
        Brain.Screen.drawLine(x1, y1, x2, y2);
        wait(50, msec); // 慢速绘制便于观察
    }

    // 显示当前位姿信息
    Brain.Screen.setPenColor(white);
    Brain.Screen.setCursor(1, 30);
    Brain.Screen.print("x:%.2f cm", currentPose.x);
    Brain.Screen.setCursor(2, 30);
    Brain.Screen.print("y:%.2f cm", currentPose.y);
    Brain.Screen.setCursor(3, 30);
    Brain.Screen.print("theta:%.2f deg", currentPose.theta * 180 / M_PI);
}

// ========================
// 其他说明
// ========================
/*
以下为 VEX V5 常用关键词（可能是你项目中的其他部分）：
config, robot, auto, user, control, function, tool, variable,
competition, autonomous, intake, spin, forward, reverse,
percent, position, set, get, void, error, volt, value
*/
