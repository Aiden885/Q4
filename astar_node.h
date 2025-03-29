// astar_node.h
#ifndef ASTAR_NODE_H
#define ASTAR_NODE_H

#include "position.h"

// A*算法使用的节点结构
struct AStarNode {
    Position pos;
    int time;
    int g_cost;  // 从起点到当前节点的实际代价
    int h_cost;  // 从当前节点到目标的估计代价
    int f_cost;  // f = g + h
    Position parent_pos;
    int parent_time;

    // 比较函数，用于优先队列排序
    bool operator>(const AStarNode& other) const {
        if (f_cost == other.f_cost) {
            return h_cost > other.h_cost;
        }
        return f_cost > other.f_cost;
    }
};

#endif // ASTAR_NODE_H