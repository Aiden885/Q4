// planner.h
#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <string>
#include <sstream>
#include "position.h"
#include "space_time_node.h"
#include "astar_node.h"
#include "robot.h"

// 多机器人路径规划类
class MultiRobotPathPlanner {
private:
    int rows, cols;
    std::vector<std::vector<int>> grid;
    std::vector<Robot> robots;
    std::unordered_set<SpaceTimeNode> occupied_nodes;  // 记录已被占用的时空节点

    // 可能的移动方向（8方向 + 原地不动）
    const std::vector<std::pair<int, int>> directions = {
        {0, 0},   // 原地不动
        {-1, -1}, // 左上
        {-1, 0},  // 上
        {-1, 1},  // 右上
        {0, -1},  // 左
        {0, 1},   // 右
        {1, -1},  // 左下
        {1, 0},   // 下
        {1, 1}    // 右下
    };

    // 初始化机器人和目标点信息
    bool initializeRobotsAndGoals();

    // 估计两点间的距离（使用曼哈顿距离）
    int calculateHeuristic(const Position& pos, const Position& goal);

    // 检查位置是否在地图范围内且不是障碍物
    bool isValidPosition(const Position& pos);

    // 检查时空节点是否已被占用
    bool isOccupied(const SpaceTimeNode& node);

    // 使用A*算法为单个机器人规划路径
    bool planPathForRobot(Robot& robot);

public:
    // 构造函数
    MultiRobotPathPlanner();

    // 从文件中读取栅格地图
    bool readGridFromFile(const std::string& filename);

    // 为所有机器人规划路径
    bool planPathsForAllRobots();

    // 输出所有机器人的路径
    void printPaths();

    // 将路径保存到文件
    bool savePathsToFile(const std::string& filename);

    // 可视化路径（以ASCII字符的形式）
    void visualizePaths();
};

#endif // PLANNER_H