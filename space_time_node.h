// space_time_node.h
#ifndef SPACE_TIME_NODE_H
#define SPACE_TIME_NODE_H

#include "position.h"

// ʱ�սڵ�ṹ������λ�ú�ʱ����Ϣ
struct SpaceTimeNode {
    Position pos;
    int time;

    // ������������
    bool operator==(const SpaceTimeNode& other) const {
        return pos == other.pos && time == other.time;
    }
};

// �Զ���SpaceTimeNode�Ĺ�ϣ����
namespace std {
    template <>
    struct hash<SpaceTimeNode> {
        size_t operator()(const SpaceTimeNode& node) const {
            return hash<Position>()(node.pos) ^ (hash<int>()(node.time) << 1);
        }
    };
}

#endif // SPACE_TIME_NODE_H