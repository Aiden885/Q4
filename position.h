// position.h
#ifndef POSITION_H
#define POSITION_H

// ����դ���ͼ�е�����ṹ
struct Position {
    int x, y;

    // ������������������Ƚ�����λ���Ƿ���ͬ
    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }

    // ���ز��������
    bool operator!=(const Position& other) const {
        return !(*this == other);
    }
};

// �Զ����ϣ����������unordered_map��unordered_set�е�Position���ͼ�
namespace std {
    template <>
    struct hash<Position> {
        size_t operator()(const Position& pos) const {
            return hash<int>()(pos.x) ^ (hash<int>()(pos.y) << 1);
        }
    };
}

#endif // POSITION_H