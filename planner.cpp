// planner.cpp
#include "planner.h"

// ���캯��
MultiRobotPathPlanner::MultiRobotPathPlanner() : rows(0), cols(0) {}

// Ϊ���л����˹滮·��
bool MultiRobotPathPlanner::planPathsForAllRobots() {
    occupied_nodes.clear();

    // �����ȼ�Ϊÿ�������˹滮·��
    // ���Ը�������ʽ����������㵽Ŀ��ľ��룩��ȷ�����ȼ�
    std::sort(robots.begin(), robots.end(), [this](const Robot& a, const Robot& b) {
        int dist_a = calculateHeuristic(a.start, a.goal);
        int dist_b = calculateHeuristic(b.start, b.goal);
        return dist_a > dist_b;  // �����Զ�Ļ��������ȹ滮
        });

    for (auto& robot : robots) {
        if (!planPathForRobot(robot)) {
            return false;
        }
    }

    return true;
}

// ������л����˵�·��
void MultiRobotPathPlanner::printPaths() {
    for (const auto& robot : robots) {
        std::cout << "������ " << robot.id << " ��·��:" << std::endl;
        for (size_t i = 0; i < robot.path.size(); ++i) {
            std::cout << "���� " << i << ": (" << robot.path[i].x << ", " << robot.path[i].y << ")" << std::endl;
        }
        std::cout << std::endl;
    }
}

// ��·�����浽�ļ�
bool MultiRobotPathPlanner::savePathsToFile(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "�޷����ļ�: " << filename << std::endl;
        return false;
    }

    // �ҳ�����·���е���󳤶�
    size_t max_path_length = 0;
    for (const auto& robot : robots) {
        max_path_length = std::max(max_path_length, robot.path.size());
    }

    // ����ÿ��ʱ�䲽
    for (size_t t = 0; t < max_path_length; ++t) {
        file << "ʱ�䲽 " << t << ":" << std::endl;

        // ������ǰʱ�䲽�ĵ�ͼ����
        std::vector<std::vector<int>> current_grid = grid;

        // ���µ�ͼ�ϵĻ�����λ��
        for (const auto& robot : robots) {
            if (t < robot.path.size()) {
                Position pos = robot.path[t];
                current_grid[pos.y][pos.x] = 10 + robot.id;  // ��10+id��ʾ������
            }
        }

        // �����ǰ��ͼ״̬
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

// ���ӻ�·������ASCII�ַ�����ʽ��
void MultiRobotPathPlanner::visualizePaths() {
    // �ҳ�����·���е���󳤶�
    size_t max_path_length = 0;
    for (const auto& robot : robots) {
        max_path_length = std::max(max_path_length, robot.path.size());
    }

    // ����ÿ��ʱ�䲽
    for (size_t t = 0; t < max_path_length; ++t) {
        std::cout << "ʱ�䲽 " << t << ":" << std::endl;

        // ������ǰʱ�䲽�ĵ�ͼ����
        std::vector<std::vector<int>> current_grid = grid;

        // ���µ�ͼ�ϵĻ�����λ��
        for (const auto& robot : robots) {
            if (t < robot.path.size()) {
                Position pos = robot.path[t];
                current_grid[pos.y][pos.x] = 10 + robot.id;  // ��10+id��ʾ������
            }
        }

        // �����ǰ��ͼ״̬
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                int value = current_grid[i][j];
                if (value == 0) {
                    std::cout << ".  ";  // �հ�
                }
                else if (value == 1) {
                    std::cout << "#  ";  // �ϰ���
                }
                else if (value >= 11 && value < 20) {
                    std::cout << "R" << (value - 10) << " ";  // ������
                }
                else if (value >= 21 && value < 30) {
                    std::cout << "G" << (value - 20) << " ";  // Ŀ��
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

// ���ļ��ж�ȡդ���ͼ
bool MultiRobotPathPlanner::readGridFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "�޷����ļ�: " << filename << std::endl;
        return false;
    }

    std::string line;
    grid.clear();
    robots.clear();

    // ��ȡÿһ��
    while (std::getline(file, line)) {
        std::vector<int> row;
        std::istringstream iss(line);
        int value;

        // ��ȡ���е�ÿ��ֵ
        while (iss >> value) {
            row.push_back(value);
        }

        if (!row.empty()) {
            grid.push_back(row);
        }
    }

    file.close();

    if (grid.empty()) {
        std::cerr << "�ļ�Ϊ�ջ��ʽ����ȷ" << std::endl;
        return false;
    }

    rows = grid.size();
    cols = grid[0].size();

    // ��鲢��ʼ�������˺�Ŀ���
    return initializeRobotsAndGoals();
}

// ��ʼ�������˺�Ŀ�����Ϣ
bool MultiRobotPathPlanner::initializeRobotsAndGoals() {
    // ����������ID��������ӳ��
    std::unordered_map<int, int> robot_indices;
    std::unordered_map<int, Position> robot_positions;
    std::unordered_map<int, Position> goal_positions;

    // ɨ���ͼ�ҳ����л����˺�Ŀ��λ��
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int value = grid[i][j];

            // ʶ���������� (11 ~ 11+n-1)
            if (value >= 11 && value < 20) {
                int robot_id = value - 10;  // ���磬11��ʾ������1
                robot_positions[robot_id] = { j, i };  // ע�⣺x��Ӧ��j��y��Ӧ��i
            }
            // ʶ��Ŀ��� (21 ~ 21+n-1)
            else if (value >= 21 && value < 30) {
                int goal_id = value - 20;  // ���磬21��ʾĿ��1
                goal_positions[goal_id] = { j, i };
            }
        }
    }

    // ȷ��ÿ�������˶��ж�Ӧ��Ŀ���
    int robot_count = robot_positions.size();
    for (int id = 1; id <= robot_count; ++id) {
        if (robot_positions.find(id) == robot_positions.end()) {
            std::cerr << "ȱ�ٻ����� " << id << " �����λ��" << std::endl;
            return false;
        }
        if (goal_positions.find(id) == goal_positions.end()) {
            std::cerr << "ȱ�ٻ����� " << id << " ��Ŀ��λ��" << std::endl;
            return false;
        }

        // ���������˶���
        Robot robot;
        robot.id = id;
        robot.start = robot_positions[id];
        robot.goal = goal_positions[id];
        robots.push_back(robot);
    }

    return true;
}

// ���������ľ��루ʹ�������پ��룩
int MultiRobotPathPlanner::calculateHeuristic(const Position& pos, const Position& goal) {
    return std::max(std::abs(pos.x - goal.x), std::abs(pos.y - goal.y));
}

// ���λ���Ƿ��ڵ�ͼ��Χ���Ҳ����ϰ���
bool MultiRobotPathPlanner::isValidPosition(const Position& pos) {
    if (pos.x < 0 || pos.x >= cols || pos.y < 0 || pos.y >= rows) {
        return false;
    }
    return grid[pos.y][pos.x] != 1;  // ֵΪ1��ʾ�ϰ���
}

// ���ʱ�սڵ��Ƿ��ѱ�ռ��
bool MultiRobotPathPlanner::isOccupied(const SpaceTimeNode& node) {
    return occupied_nodes.find(node) != occupied_nodes.end();
}

// ʹ��A*�㷨Ϊ���������˹滮·��
bool MultiRobotPathPlanner::planPathForRobot(Robot& robot) {
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
    std::unordered_set<SpaceTimeNode> closed_list;
    std::unordered_map<SpaceTimeNode, std::pair<Position, int>> came_from;

    // ��ʼ�ڵ�
    AStarNode start_node;
    start_node.pos = robot.start;
    start_node.time = 0;
    start_node.g_cost = 0;
    start_node.h_cost = calculateHeuristic(robot.start, robot.goal);
    start_node.f_cost = start_node.g_cost + start_node.h_cost;

    open_list.push(start_node);

    // ���ʱ�䲽�����ƣ���ֹ����ѭ��
    int max_time_steps = rows * cols * 3;

    while (!open_list.empty()) {
        // ��ȡfֵ��С�Ľڵ�
        AStarNode current = open_list.top();
        open_list.pop();

        SpaceTimeNode current_st_node = { current.pos, current.time };

        // ����Ѿ���������ʱ�սڵ㣬������
        if (closed_list.find(current_st_node) != closed_list.end()) {
            continue;
        }

        // ����ǰ�ڵ���ӵ��ر��б�
        closed_list.insert(current_st_node);

        // ��¼��Դ�ڵ�
        if (current.time > 0) {
            came_from[current_st_node] = { current.parent_pos, current.parent_time };
        }

        // ����Ŀ��λ��
        if (current.pos == robot.goal) {
            // �ؽ�·��
            std::vector<Position> path;
            SpaceTimeNode backtrack_node = current_st_node;

            while (backtrack_node.time > 0) {
                path.push_back(backtrack_node.pos);
                auto parent = came_from[backtrack_node];
                backtrack_node = { parent.first, parent.second };
            }

            path.push_back(robot.start);
            std::reverse(path.begin(), path.end());

            // ��·�������������
            robot.path = path;

            // ���·���ϵ�����ʱ�սڵ�Ϊ��ռ��
            for (size_t t = 0; t < path.size(); ++t) {
                occupied_nodes.insert({ path[t], static_cast<int>(t) });
            }

            return true;
        }

        // �������ʱ�䲽��
        if (current.time >= max_time_steps) {
            continue;
        }

        // �������п��ܵ��ƶ�����
        for (const auto& dir : directions) {
            Position next_pos = { current.pos.x + dir.first, current.pos.y + dir.second };
            int next_time = current.time + 1;

            // �����һ��λ���Ƿ���Ч
            if (!isValidPosition(next_pos)) {
                continue;
            }

            SpaceTimeNode next_st_node = { next_pos, next_time };

            // ����Ƿ������������˷�����ײ
            if (isOccupied(next_st_node)) {
                continue;
            }

            // ���·���������
            bool has_conflict = false;
            for (const auto& other_robot : robots) {
                // �����Լ�
                if (other_robot.id == robot.id || other_robot.path.empty()) {
                    continue;
                }

                // ����Ƿ���������������ʱ��t-1��t֮�䷢��·������
                if (next_time > 0 && next_time <= static_cast<int>(other_robot.path.size())) {
                    Position other_pos_prev = other_robot.path[next_time - 1];
                    Position other_pos_curr = other_robot.path[next_time >= static_cast<int>(other_robot.path.size()) ?
                        other_robot.path.size() - 1 : next_time];

                    // �����ǰ�����˴�current.pos�ƶ���next_pos������һ�������˴�other_pos_prev�ƶ���other_pos_curr
                    // �������ǵ�·�����棬����ڳ�ͻ
                    if (current.pos == other_pos_curr && next_pos == other_pos_prev) {
                        has_conflict = true;
                        break;
                    }
                }
            }

            if (has_conflict) {
                continue;
            }

            // ������ʱ�սڵ��Ѿ��ڹر��б��У�������
            if (closed_list.find(next_st_node) != closed_list.end()) {
                continue;
            }

            // �������
            int next_g_cost = current.g_cost + 1;
            int next_h_cost = calculateHeuristic(next_pos, robot.goal);
            int next_f_cost = next_g_cost + next_h_cost;

            // ������һ���ڵ�
            AStarNode next_node;
            next_node.pos = next_pos;
            next_node.time = next_time;
            next_node.g_cost = next_g_cost;
            next_node.h_cost = next_h_cost;
            next_node.f_cost = next_f_cost;
            next_node.parent_pos = current.pos;
            next_node.parent_time = current.time;

            // ���ڵ���ӵ������б�
            open_list.push(next_node);
        }
    }

    // ����޷��ҵ�·��
    std::cerr << "�޷�Ϊ������ " << robot.id << " �ҵ���Ч·��" << std::endl;
    return false;
}