// robot.h
#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include "position.h"

// 机器人信息结构
struct Robot {
    int id;
    Position start;
    Position goal;
    std::vector<Position> path;
};

#endif // ROBOT_H