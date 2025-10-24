#include "vex.h"

/**
 * 左侧自动程序（从场地左侧出发）
 */
void stoplr(){
  L.stop(brake);
  R.stop(brake);
}

void Left(){
  // 设置升降机构（Export）停止方式为 hold（保持当前位置）
  Export.setStopping(hold);
  // 设置左右驱动轮停止方式为 brake（刹车）
  L.setStopping(brake);
  R.setStopping(brake);

  // 启动主 intake（吸球机构）正转，100% 功率
  Intake.spin(forward, 100, vex::velocityUnits::pct);
  // 启动副 intake（可能是辅助滚轮）正转，60% 功率
  Intake2.spin(forward, 60, vex::velocityUnits::pct);

  // 平滑前进至中桥区域（带 PID 转向控制）
  // 参数说明：方向、距离(mm)、最高速度(%)、开始减速距离(mm)、最低速度(%)、目标朝向角度、KP（比例系数）
  // linearSmoothStop(fwd, 1300, 30, 150, 20, 338, 0.5);

  // 继续前进 400mm，速度 40%
  // move(fwd, 1300, 80);
  linearSmoothStop(fwd, 1900, 80, 800, 80, 340, 0.5);
  linearSmoothStop(fwd, 1900, 80, 800, 50, 0, 0.5);
  Intake.stop();
  Intake2.stop();
  linearSmoothStop(reverse, 500, 80, 800, 50, 30, 0.5);
  // 稍微后退 200mm，速度 20%，可能是微调位置
  // move(reverse, 200, 20);
  // wait(100, msec); // 等待 100 毫秒，确保动作完成

  // —————— 到达中桥区域 ——————

  // 智能转向：向左转到 -130 度（等效 230 度），参数为角度、KP、KI、KD（PID 参数）
  // smartTurn(-130, 0.48, 0.07, 0.01);

  // 后退 700mm，速度 30%，准备对准中桥入口
  // move(fwd, 700, 30);

  // 微调后退（左右轮各以 4% 速度反转），用于精细对准
  // L.spin(reverse, 4, vex::velocityUnits::pct);
  // R.spin(reverse, 4, vex::velocityUnits::pct);

  // 将升降机构停止方式改为 coast（自由滑行），避免卡住
  // Export.setStopping(coast);

  // 关闭副 intake，防止在中桥投放时吸入多余球
  // Intake2.stop();

  // 升降机构反转 0.7 圈，速度 50%，将球推出中桥
  // Export.spinFor(reverse, 0.7, turns, 50, vex::velocityUnits::pct);

  // 停止驱动轮
  // L.stop(brake);
  // R.stop(brake);

  // 升降机构轻微正转 0.1 圈，防止卡住或复位
  // Export.spinFor(fwd, 0.1, turns, 100, vex::velocityUnits::pct);

  // 从中桥后退并前往得分桶区域，带转向控制（目标角度 226°）
  // linearSmoothStop(fwd, 1900, 80, 800, 50, 226, 0.5);
  // wait(100, msec); // 等待稳定

  // 转向至 180°（正对得分桶）
  // smartTurn(180, 0.54, 0.09, 0.16);

  // 调用函数：将球从桶区域推入得分区（具体实现未在此文件）
  // Bucket_to_Bridge();
}
