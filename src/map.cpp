// Map.cpp
#include "vex.h"
#include <vector>
#include <cmath>

using namespace vex;

// 场地地图定义 (以英寸为单位)
const double FIELD_WIDTH = 144.0;    // 场地宽度
const double FIELD_LENGTH = 144.0;   // 场地长度

// 关键场地元素位置 (根据比赛场地定义)
struct FieldElement {
    double x, y;        // 位置坐标
    double width, height; // 尺寸
    const char* name;   // 元素名称
};

// 场地元素列表 (需要根据实际比赛场地更新)
std::vector<FieldElement> field_elements = {
    // 目标区域
    {72.0, 12.0, 24.0, 24.0, "Red Goal"},
    {72.0, 132.0, 24.0, 24.0, "Blue Goal"},
    
    // 障碍物或重要地标
    {36.0, 36.0, 12.0, 12.0, "Obstacle 1"},
    {108.0, 36.0, 12.0, 12.0, "Obstacle 2"},
    {36.0, 108.0, 12.0, 12.0, "Obstacle 3"},
    {108.0, 108.0, 12.0, 12.0, "Obstacle 4"}
};

// 场地边界
struct FieldBoundary {
    double x1, y1, x2, y2;
};

std::vector<FieldBoundary> field_boundaries = {
    {0, 0, FIELD_LENGTH, 0},         // 下边界
    {0, FIELD_WIDTH, FIELD_LENGTH, FIELD_WIDTH}, // 上边界
    {0, 0, 0, FIELD_WIDTH},          // 左边界
    {FIELD_LENGTH, 0, FIELD_LENGTH, FIELD_WIDTH} // 右边界
};

/*
 * 检查位置是否在场地内
 */
bool isInField(double x, double y, double robot_radius = 9.0) {
    return (x >= robot_radius && x <= FIELD_LENGTH - robot_radius &&
            y >= robot_radius && y <= FIELD_WIDTH - robot_radius);
}

/*
 * 计算到最近场边界的距离
 */
// double distanceToNearestBoundary(double x, double y) {
    double min_distance = FIELD_LENGTH;
    
    for (const auto& boundary : field_boundaries) {
        // 简化计算：点到线段的距离
        double dx = boundary.x2 - boundary.x1;
        double dy = boundary.y2 - boundary.y1;
        
        if (fabs(dx) < 1e-6 && fabs(dy) < 1e-6) {
            // 线段退化为点
            double distance = hypot(x - boundary.x1, y - boundary.y1);
            min_distance = fmin(min_distance, distance);
        } else {
            double t = ((x - boundary.x1) * dx + (y - boundary.y1) * dy) / (dx * dx + dy * dy);
            t = fmax(0.0, fmin(1.0, t));
            
            double closest_x = boundary.x1 + t * dx;
            double closest_y = boundary.y1 + t * dy;
            
            double distance = hypot(x - closest_x, y - closest_y);
            min_distance = fmin(min_distance, distance);
        }
    }
    
    return min_distance;
}

/*
 * 获取最近的场地元素
 */
FieldElement getNearestElement(double x, double y) {
    double min_distance = FIELD_LENGTH * 2;
    FieldElement nearest = field_elements[0];
    
    for (const auto& element : field_elements) {
        // 计算到元素中心的距离
        double element_center_x = element.x + element.width / 2;
        double element_center_y = element.y + element.height / 2;
        
        double distance = hypot(x - element_center_x, y - element_center_y);
        
        if (distance < min_distance) {
            min_distance = distance;
            nearest = element;
        }
    }
    
    return nearest;
}

/*
 * 显示场地地图和机器人位置
 */
void displayMap(double robot_x, double robot_y, double robot_heading) {
    Brain.Screen.clearScreen(black);
    
    // 绘制场地边界
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.drawRectangle(10, 10, 200, 200);
    
    // 坐标缩放因子
    double scale_x = 200.0 / FIELD_LENGTH;
    double scale_y = 200.0 / FIELD_WIDTH;
    
    // 绘制场地元素
    Brain.Screen.setPenColor(red);
    for (const auto& element : field_elements) {
        int screen_x = 10 + (int)(element.x * scale_x);
        int screen_y = 10 + (int)(element.y * scale_y);
        int screen_width = (int)(element.width * scale_x);
        int screen_height = (int)(element.height * scale_y);
        
        Brain.Screen.drawRectangle(screen_x, screen_y, screen_width, screen_height);
    }
    
    // 绘制机器人位置
    int robot_screen_x = 10 + (int)(robot_x * scale_x);
    int robot_screen_y = 10 + (int)(robot_y * scale_y);
    
    Brain.Screen.setPenColor(green);
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawCircle(robot_screen_x, robot_screen_y, 5);
    
    // 绘制机器人朝向
    double arrow_length = 15.0;
    double arrow_x = robot_screen_x + arrow_length * sin(robot_heading * M_PI / 180.0);
    double arrow_y = robot_screen_y - arrow_length * cos(robot_heading * M_PI / 180.0);
    
    Brain.Screen.drawLine(robot_screen_x, robot_screen_y, arrow_x, arrow_y);
    
    // 显示坐标信息
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(220, 30, "X: %.1f", robot_x);
    Brain.Screen.printAt(220, 50, "Y: %.1f", robot_y);
    Brain.Screen.printAt(220, 70, "Heading: %.1f", robot_heading);
    
    // 显示最近的场地元素
    FieldElement nearest = getNearestElement(robot_x, robot_y);
    Brain.Screen.printAt(220, 100, "Near: %s", nearest.name);
}

/*
 * 重置场地坐标
 */
void resetFieldCoordinates() {
    // 可以根据需要设置不同的起始位置
    // 例如：从场地角落开始
}