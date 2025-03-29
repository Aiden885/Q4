// space_time_node.h
#ifndef SPACE_TIME_NODE_H
#define SPACE_TIME_NODE_H

#include "position.h"

// 时空节点结构，包含位置和时间信息
struct SpaceTimeNode {
    Position pos;
    int time;

    // 重载相等运算符
    bool operator==(const SpaceTimeNode& other) const {
        return pos == other.pos && time == other.time;
    }
};

// 自定义SpaceTimeNode的哈希函数
namespace std {
    template <>
    struct hash<SpaceTimeNode> {
        size_t operator()(const SpaceTimeNode& node) const {
            return hash<Position>()(node.pos) ^ (hash<int>()(node.time) << 1);
        }
    };
}

#endif // SPACE_TIME_NODE_H