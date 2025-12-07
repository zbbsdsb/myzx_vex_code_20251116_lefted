// FieldMap.h
#ifndef FIELD_MAP_H
#define FIELD_MAP_H

// 简单的场地定义
class FieldMap {
public:
    static const double WIDTH = 144.0;   // 场地宽度 (英寸)
    static const double LENGTH = 144.0;  // 场地长度 (英寸)
    
    // 检查位置是否在场地内
    static bool isInField(double x, double y, double margin = 9.0) {
        return (x >= margin && x <= LENGTH - margin &&
                y >= margin && y <= WIDTH - margin);
    }
    
    // 显示场地和机器人位置
    static void display(double robot_x, double robot_y, double robot_heading);
};

#endif