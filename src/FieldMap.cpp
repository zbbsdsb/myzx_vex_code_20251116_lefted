// FieldMap.cpp
#include "FieldMap.h"
#include "vex.h"

using namespace vex;

void FieldMap::display(double robot_x, double robot_y, double robot_heading) {
    Brain.Screen.clearScreen(black);
    
    // 场地缩放
    double scale_x = 200.0 / LENGTH;
    double scale_y = 200.0 / WIDTH;
    
    // 绘制场地边界
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawRectangle(10, 10, 200, 200);
    
    // 绘制机器人位置
    int screen_x = 10 + (int)(robot_x * scale_x);
    int screen_y = 10 + (int)(robot_y * scale_y);
    
    Brain.Screen.setPenColor(green);
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawCircle(screen_x, screen_y, 5);
    
    // 绘制朝向
    double arrow_len = 15.0;
    double arrow_x = screen_x + arrow_len * sin(robot_heading * M_PI / 180.0);
    double arrow_y = screen_y - arrow_len * cos(robot_heading * M_PI / 180.0);
    Brain.Screen.drawLine(screen_x, screen_y, arrow_x, arrow_y);
    
    // 显示坐标
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(220, 30, "X: %.1f", robot_x);
    Brain.Screen.printAt(220, 50, "Y: %.1f", robot_y);
    Brain.Screen.printAt(220, 70, "H: %.1f°", robot_heading);
    
    // 检查边界
    if(!isInField(robot_x, robot_y)) {
        Brain.Screen.setPenColor(red);
        Brain.Screen.printAt(220, 90, "OUT OF FIELD!");
    }
}