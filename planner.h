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

// �������·���滮��
class MultiRobotPathPlanner {
private:
    int rows, cols;
    std::vector<std::vector<int>> grid;
    std::vector<Robot> robots;
    std::unordered_set<SpaceTimeNode> occupied_nodes;  // ��¼�ѱ�ռ�õ�ʱ�սڵ�

    // ���ܵ��ƶ�����8���� + ԭ�ز�����
    const std::vector<std::pair<int, int>> directions = {
        {0, 0},   // ԭ�ز���
        {-1, -1}, // ����
        {-1, 0},  // ��
        {-1, 1},  // ����
        {0, -1},  // ��
        {0, 1},   // ��
        {1, -1},  // ����
        {1, 0},   // ��
        {1, 1}    // ����
    };

    // ��ʼ�������˺�Ŀ�����Ϣ
    bool initializeRobotsAndGoals();

    // ���������ľ��루ʹ�������پ��룩
    int calculateHeuristic(const Position& pos, const Position& goal);

    // ���λ���Ƿ��ڵ�ͼ��Χ���Ҳ����ϰ���
    bool isValidPosition(const Position& pos);

    // ���ʱ�սڵ��Ƿ��ѱ�ռ��
    bool isOccupied(const SpaceTimeNode& node);

    // ʹ��A*�㷨Ϊ���������˹滮·��
    bool planPathForRobot(Robot& robot);

public:
    // ���캯��
    MultiRobotPathPlanner();

    // ���ļ��ж�ȡդ���ͼ
    bool readGridFromFile(const std::string& filename);

    // Ϊ���л����˹滮·��
    bool planPathsForAllRobots();

    // ������л����˵�·��
    void printPaths();

    // ��·�����浽�ļ�
    bool savePathsToFile(const std::string& filename);

    // ���ӻ�·������ASCII�ַ�����ʽ��
    void visualizePaths();
};

#endif // PLANNER_H