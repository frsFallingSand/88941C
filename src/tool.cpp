#include "vex.h"

float TargetAngle;

//陀螺仪角度曲线图
void IMU_Display(){
   // 清空屏幕
    Brain.Screen.clearScreen();
    // 设置图形显示区域
    int graphWidth = 400;
    int graphHeight = 200;
    int graphX = 5;
    int graphY = 20;
    // 绘制坐标轴
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawRectangle(graphX, graphY, graphWidth, graphHeight);
    
    // 绘制中心线 (0度线)
    Brain.Screen.setPenColor(green);
    int zeroY = graphY + graphHeight/2;
    Brain.Screen.drawLine(graphX, zeroY, graphX + graphWidth, zeroY);
    // 变量用于存储历史数据
    const int maxPoints = 50;
    float angleHistory[maxPoints] = {0};
    int currentIndex = 0;
    while(true) {
        // 获取当前陀螺仪角度
        float currentAngle = IMU.rotation(degrees);
        
        // 存储当前角度到历史数组
        angleHistory[currentIndex] = currentAngle;
        currentIndex = (currentIndex + 1) % maxPoints;
        // 清空图形区域
        Brain.Screen.setPenColor(black);
        Brain.Screen.setFillColor(black);
        Brain.Screen.drawRectangle(graphX + 1, graphY + 1, graphWidth - 2, graphHeight - 2);
        // 重新绘制中心线
        Brain.Screen.setPenColor(green);
        Brain.Screen.drawLine(graphX, zeroY, graphX + graphWidth, zeroY);
        // 绘制角度曲线
        Brain.Screen.setPenColor(blue);
        
        for(int i = 0; i < maxPoints - 1; i++) {
            int idx1 = (currentIndex + i) % maxPoints;
            int idx2 = (currentIndex + i + 1) % maxPoints;
            
            if(angleHistory[idx1] != 0 || angleHistory[idx2] != 0) {
                int x1 = graphX + (i * graphWidth / maxPoints);
                int x2 = graphX + ((i + 1) * graphWidth / maxPoints);
                int y1 = zeroY - ((angleHistory[idx1]-TargetAngle)* graphHeight / 180); // 缩放因子
                int y2 = zeroY - ((angleHistory[idx2]-TargetAngle)* graphHeight / 180);
                
                Brain.Screen.drawLine(x1, y1, x2, y2);
            }
        }
        // 显示当前角度值
        Brain.Screen.setPenColor(white);
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Current Angle: %.2f degrees", currentAngle);
        Brain.Screen.setCursor(3, 42);
        Brain.Screen.print("P: %.2f", P);
        Brain.Screen.setCursor(5, 42);
        Brain.Screen.print("I: %.2f", I);
        Brain.Screen.setCursor(7, 42);
        Brain.Screen.print("D: %.2f", D);
        Brain.Screen.setCursor(9, 42);
        Brain.Screen.print("F: %.2f", feedforward);
        wait(50, msec); // 控制刷新率
    }
}


void feedforward_kV(){
  // 保持恒定速度运行
  L.setStopping(brake);
  R.setStopping(brake);
  for(double speed = 10; speed <= 100; speed += 10) {
      L.spin(forward, speed, percent);
      wait(2, seconds);
      double actualSpeed = L.velocity(rpm);
      double kV = speed / actualSpeed; // 更新系数
      Brain.Screen.setCursor(1,1);
      Brain.Screen.print("kV: %.2f", kV);
      wait(10,sec);
    }
  }

void feedforward_kA(){
  // 测量加速时间
  double startTime = Brain.timer(msec);
  L.spin(forward, 100, percent);
  while(L.velocity(rpm) < 300) wait(5, msec);
  double accelTime = Brain.timer(msec) - startTime;
  double kA = 100 / (300/accelTime); // 电压/(rpm/s)
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("kA: %.2f", kA);
  wait(2,sec);
  L.stop();
  wait(10,sec);
}


