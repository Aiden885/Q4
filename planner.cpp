// planner.cpp
#include "planner.h"

// 构造函数
MultiRobotPathPlanner::MultiRobotPathPlanner() : rows(0), cols(0) {}

// 为所有机器人规划路径
bool MultiRobotPathPlanner::planPathsForAllRobots() {
    occupied_nodes.clear();

    // 按优先级为每个机器人规划路径
    // 可以根据启发式函数（如起点到目标的距离）来确定优先级
    std::sort(robots.begin(), robots.end(), [this](const Robot& a, const Robot& b) {
        int dist_a = calculateHeuristic(a.start, a.goal);
        int dist_b = calculateHeuristic(b.start, b.goal);
        return dist_a > dist_b;  // 距离更远的机器人优先规划
        });

    for (auto& robot : robots) {
        if (!planPathForRobot(robot)) {
            return false;
        }
    }

    return true;
}

// 输出所有机器人的路径
void MultiRobotPathPlanner::printPaths() {
    for (const auto& robot : robots) {
        std::cout << "机器人 " << robot.id << " 的路径:" << std::endl;
        for (size_t i = 0; i < robot.path.size(); ++i) {
            std::cout << "步骤 " << i << ": (" << robot.path[i].x << ", " << robot.path[i].y << ")" << std::endl;
        }
        std::cout << std::endl;
    }
}

// 将路径保存到文件
bool MultiRobotPathPlanner::savePathsToFile(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }

    // 找出所有路径中的最大长度
    size_t max_path_length = 0;
    for (const auto& robot : robots) {
        max_path_length = std::max(max_path_length, robot.path.size());
    }

    // 对于每个时间步
    for (size_t t = 0; t < max_path_length; ++t) {
        file << "时间步 " << t << ":" << std::endl;

        // 创建当前时间步的地图副本
        std::vector<std::vector<int>> current_grid = grid;

        // 更新地图上的机器人位置
        for (const auto& robot : robots) {
            if (t < robot.path.size()) {
                Position pos = robot.path[t];
                current_grid[pos.y][pos.x] = 10 + robot.id;  // 用10+id表示机器人
            }
        }

        // 输出当前地图状态
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                file << current_grid[i][j] << "\t";
            }
            file << std::endl;
        }
        file << std::endl;
    }

    file.close();
    return true;
}

// 可视化路径（以ASCII字符的形式）
void MultiRobotPathPlanner::visualizePaths() {
    // 找出所有路径中的最大长度
    size_t max_path_length = 0;
    for (const auto& robot : robots) {
        max_path_length = std::max(max_path_length, robot.path.size());
    }

    // 对于每个时间步
    for (size_t t = 0; t < max_path_length; ++t) {
        std::cout << "时间步 " << t << ":" << std::endl;

        // 创建当前时间步的地图副本
        std::vector<std::vector<int>> current_grid = grid;

        // 更新地图上的机器人位置
        for (const auto& robot : robots) {
            if (t < robot.path.size()) {
                Position pos = robot.path[t];
                current_grid[pos.y][pos.x] = 10 + robot.id;  // 用10+id表示机器人
            }
        }

        // 输出当前地图状态
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                int value = current_grid[i][j];
                if (value == 0) {
                    std::cout << ".  ";  // 空白
                }
                else if (value == 1) {
                    std::cout << "#  ";  // 障碍物
                }
                else if (value >= 11 && value < 20) {
                    std::cout << "R" << (value - 10) << " ";  // 机器人
                }
                else if (value >= 21 && value < 30) {
                    std::cout << "G" << (value - 20) << " ";  // 目标
                }
                else {
                    std::cout << value << " ";
                }
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
}

// 从文件中读取栅格地图
bool MultiRobotPathPlanner::readGridFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }

    std::string line;
    grid.clear();
    robots.clear();

    // 读取每一行
    while (std::getline(file, line)) {
        std::vector<int> row;
        std::istringstream iss(line);
        int value;

        // 读取行中的每个值
        while (iss >> value) {
            row.push_back(value);
        }

        if (!row.empty()) {
            grid.push_back(row);
        }
    }

    file.close();

    if (grid.empty()) {
        std::cerr << "文件为空或格式不正确" << std::endl;
        return false;
    }

    rows = grid.size();
    cols = grid[0].size();

    // 检查并初始化机器人和目标点
    return initializeRobotsAndGoals();
}

// 初始化机器人和目标点信息
bool MultiRobotPathPlanner::initializeRobotsAndGoals() {
    // 创建机器人ID到索引的映射
    std::unordered_map<int, int> robot_indices;
    std::unordered_map<int, Position> robot_positions;
    std::unordered_map<int, Position> goal_positions;

    // 扫描地图找出所有机器人和目标位置
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int value = grid[i][j];

            // 识别机器人起点 (11 ~ 11+n-1)
            if (value >= 11 && value < 20) {
                int robot_id = value - 10;  // 例如，11表示机器人1
                robot_positions[robot_id] = { j, i };  // 注意：x对应列j，y对应行i
            }
            // 识别目标点 (21 ~ 21+n-1)
            else if (value >= 21 && value < 30) {
                int goal_id = value - 20;  // 例如，21表示目标1
                goal_positions[goal_id] = { j, i };
            }
        }
    }

    // 确保每个机器人都有对应的目标点
    int robot_count = robot_positions.size();
    for (int id = 1; id <= robot_count; ++id) {
        if (robot_positions.find(id) == robot_positions.end()) {
            std::cerr << "缺少机器人 " << id << " 的起点位置" << std::endl;
            return false;
        }
        if (goal_positions.find(id) == goal_positions.end()) {
            std::cerr << "缺少机器人 " << id << " 的目标位置" << std::endl;
            return false;
        }

        // 创建机器人对象
        Robot robot;
        robot.id = id;
        robot.start = robot_positions[id];
        robot.goal = goal_positions[id];
        robots.push_back(robot);
    }

    return true;
}

// 估计两点间的距离（使用曼哈顿距离）
int MultiRobotPathPlanner::calculateHeuristic(const Position& pos, const Position& goal) {
    return std::max(std::abs(pos.x - goal.x), std::abs(pos.y - goal.y));
}

// 检查位置是否在地图范围内且不是障碍物
bool MultiRobotPathPlanner::isValidPosition(const Position& pos) {
    if (pos.x < 0 || pos.x >= cols || pos.y < 0 || pos.y >= rows) {
        return false;
    }
    return grid[pos.y][pos.x] != 1;  // 值为1表示障碍物
}

// 检查时空节点是否已被占用
bool MultiRobotPathPlanner::isOccupied(const SpaceTimeNode& node) {
    return occupied_nodes.find(node) != occupied_nodes.end();
}

// 使用A*算法为单个机器人规划路径
bool MultiRobotPathPlanner::planPathForRobot(Robot& robot) {
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
    std::unordered_set<SpaceTimeNode> closed_list;
    std::unordered_map<SpaceTimeNode, std::pair<Position, int>> came_from;

    // 初始节点
    AStarNode start_node;
    start_node.pos = robot.start;
    start_node.time = 0;
    start_node.g_cost = 0;
    start_node.h_cost = calculateHeuristic(robot.start, robot.goal);
    start_node.f_cost = start_node.g_cost + start_node.h_cost;

    open_list.push(start_node);

    // 最大时间步数限制，防止无限循环
    int max_time_steps = rows * cols * 3;

    while (!open_list.empty()) {
        // 获取f值最小的节点
        AStarNode current = open_list.top();
        open_list.pop();

        SpaceTimeNode current_st_node = { current.pos, current.time };

        // 如果已经处理过这个时空节点，则跳过
        if (closed_list.find(current_st_node) != closed_list.end()) {
            continue;
        }

        // 将当前节点添加到关闭列表
        closed_list.insert(current_st_node);

        // 记录来源节点
        if (current.time > 0) {
            came_from[current_st_node] = { current.parent_pos, current.parent_time };
        }

        // 到达目标位置
        if (current.pos == robot.goal) {
            // 重建路径
            std::vector<Position> path;
            SpaceTimeNode backtrack_node = current_st_node;

            while (backtrack_node.time > 0) {
                path.push_back(backtrack_node.pos);
                auto parent = came_from[backtrack_node];
                backtrack_node = { parent.first, parent.second };
            }

            path.push_back(robot.start);
            std::reverse(path.begin(), path.end());

            // 将路径分配给机器人
            robot.path = path;

            // 标记路径上的所有时空节点为已占用
            for (size_t t = 0; t < path.size(); ++t) {
                occupied_nodes.insert({ path[t], static_cast<int>(t) });
            }

            return true;
        }

        // 超过最大时间步数
        if (current.time >= max_time_steps) {
            continue;
        }

        // 尝试所有可能的移动方向
        for (const auto& dir : directions) {
            Position next_pos = { current.pos.x + dir.first, current.pos.y + dir.second };
            int next_time = current.time + 1;

            // 检查下一个位置是否有效
            if (!isValidPosition(next_pos)) {
                continue;
            }

            SpaceTimeNode next_st_node = { next_pos, next_time };

            // 检查是否与其他机器人发生碰撞
            if (isOccupied(next_st_node)) {
                continue;
            }

            // 检查路径交叉情况
            bool has_conflict = false;
            for (const auto& other_robot : robots) {
                // 跳过自己
                if (other_robot.id == robot.id || other_robot.path.empty()) {
                    continue;
                }

                // 检查是否与其他机器人在时间t-1到t之间发生路径交叉
                if (next_time > 0 && next_time <= static_cast<int>(other_robot.path.size())) {
                    Position other_pos_prev = other_robot.path[next_time - 1];
                    Position other_pos_curr = other_robot.path[next_time >= static_cast<int>(other_robot.path.size()) ?
                        other_robot.path.size() - 1 : next_time];

                    // 如果当前机器人从current.pos移动到next_pos，而另一个机器人从other_pos_prev移动到other_pos_curr
                    // 并且它们的路径交叉，则存在冲突
                    if (current.pos == other_pos_curr && next_pos == other_pos_prev) {
                        has_conflict = true;
                        break;
                    }
                }
            }

            if (has_conflict) {
                continue;
            }

            // 如果这个时空节点已经在关闭列表中，则跳过
            if (closed_list.find(next_st_node) != closed_list.end()) {
                continue;
            }

            // 计算代价
            int next_g_cost = current.g_cost + 1;
            int next_h_cost = calculateHeuristic(next_pos, robot.goal);
            int next_f_cost = next_g_cost + next_h_cost;

            // 创建下一个节点
            AStarNode next_node;
            next_node.pos = next_pos;
            next_node.time = next_time;
            next_node.g_cost = next_g_cost;
            next_node.h_cost = next_h_cost;
            next_node.f_cost = next_f_cost;
            next_node.parent_pos = current.pos;
            next_node.parent_time = current.time;

            // 将节点添加到开放列表
            open_list.push(next_node);
        }
    }

    // 如果无法找到路径
    std::cerr << "无法为机器人 " << robot.id << " 找到有效路径" << std::endl;
    return false;
}