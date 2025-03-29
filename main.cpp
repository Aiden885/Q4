// main.cpp
#include "planner.h"

// 主函数
int main() {
    MultiRobotPathPlanner planner;

    // 从文件读取栅格地图
    std::string input_filename = "grid_map_4.txt";
    std::cout << "读取栅格地图文件: " << input_filename << std::endl;

    if (!planner.readGridFromFile(input_filename)) {
        std::cerr << "读取栅格地图失败" << std::endl;
        return 1;
    }

    // 规划所有机器人的路径
    std::cout << "开始规划路径..." << std::endl;
    if (!planner.planPathsForAllRobots()) {
        std::cerr << "无法为所有机器人找到有效路径" << std::endl;
        return 1;
    }

    // 输出路径
    planner.printPaths();

    // 可视化路径
    planner.visualizePaths();

    // 将路径保存到文件
    std::string output_filename = "paths_result.txt";
    if (!planner.savePathsToFile(output_filename)) {
        std::cerr << "保存路径到文件失败" << std::endl;
        return 1;
    }

    std::cout << "路径规划完成，结果已保存到 " << output_filename << std::endl;

    return 0;
}