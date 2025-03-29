// position.h
#ifndef POSITION_H
#define POSITION_H

// 定义栅格地图中的坐标结构
struct Position {
    int x, y;

    // 重载相等运算符，方便比较两个位置是否相同
    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }

    // 重载不等运算符
    bool operator!=(const Position& other) const {
        return !(*this == other);
    }
};

// 自定义哈希函数，用于unordered_map和unordered_set中的Position类型键
namespace std {
    template <>
    struct hash<Position> {
        size_t operator()(const Position& pos) const {
            return hash<int>()(pos.x) ^ (hash<int>()(pos.y) << 1);
        }
    };
}

#endif // POSITION_H