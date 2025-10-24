#include "vector"

extern double clamp(double temp, double min, double max);
extern void test();
extern void usercontrol();
extern void autonomous();
extern vex::motor_group L;
extern vex::motor_group R;
extern float TargetAngle;
extern double P;
extern double I;
extern double D;
extern double feedforward;
extern int front_panel_cnt;//前挡板计数器
extern bool front_panel_state;//前挡板状态值
extern int Double_hook_cnt;//钩子计数器
extern bool Double_hook_state;//钩子状态值


//tool工具类的全局声名
extern void IMU_Display(); //惯性传感器数值绘图
extern void feedforward_kV();
extern void feedforward_kA();
extern void init();

//自动函数
extern void Right();
extern void Left();
extern void Auto();


//运动函数
extern void smartTurn(double targetAngle,double kP,double kI,double kD);
extern void move(vex:: directionType dir,float dis,int v);
extern void linearSmoothStop(vex:: directionType dir,float dis,float Maxspeed,float DecelerationDis,float Minspeed,int targetdeg,double kp);
extern void moveTime(vex:: directionType dir,double sp,double times);
extern void Bucket_to_Bridge();
extern void Bucket_to_Bridge_rdiff();
extern void MoveDistancePID(vex:: directionType dir,float targetDist,float Maxspeed,float minSpeed,float targetAngle,float DISkp,float Anglekp);



//遥控函数
extern void front_panel_control();//控制前挡板
extern void front_panel_released();//前挡板按键松开时， 调整状态值。
extern void Double_hook_control();//控制两边钩子
extern void Double_hook_released();//钩子按键松开时， 调整状态值。


//pure pursurt
// ==================== 数据结构 ====================
extern struct Point;//点结构体
extern struct Pose;//位姿结构体
extern void setup();//正交里程计&IMU初始化
extern void updateOdometry();//正交里程计与IMU融合更新

/*
贝塞尔曲线生成相关
*/
extern std::vector<Point> bezierPath;//贝塞尔曲线点集
extern Point bezierPoint(Point p0, Point p1, Point p2, Point p3, double t);//生成
extern std::vector<Point> generateBezierPath(Point p0, Point p1, Point p2, Point p3, int num);
extern void visualizePath(const std::vector<Point>& path);