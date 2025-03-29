// astar_node.h
#ifndef ASTAR_NODE_H
#define ASTAR_NODE_H

#include "position.h"

// A*�㷨ʹ�õĽڵ�ṹ
struct AStarNode {
    Position pos;
    int time;
    int g_cost;  // ����㵽��ǰ�ڵ��ʵ�ʴ���
    int h_cost;  // �ӵ�ǰ�ڵ㵽Ŀ��Ĺ��ƴ���
    int f_cost;  // f = g + h
    Position parent_pos;
    int parent_time;

    // �ȽϺ������������ȶ�������
    bool operator>(const AStarNode& other) const {
        if (f_cost == other.f_cost) {
            return h_cost > other.h_cost;
        }
        return f_cost > other.f_cost;
    }
};

#endif // ASTAR_NODE_H