// main.cpp
#include "planner.h"

// ������
int main() {
    MultiRobotPathPlanner planner;

    // ���ļ���ȡդ���ͼ
    std::string input_filename = "grid_map_4.txt";
    std::cout << "��ȡդ���ͼ�ļ�: " << input_filename << std::endl;

    if (!planner.readGridFromFile(input_filename)) {
        std::cerr << "��ȡդ���ͼʧ��" << std::endl;
        return 1;
    }

    // �滮���л����˵�·��
    std::cout << "��ʼ�滮·��..." << std::endl;
    if (!planner.planPathsForAllRobots()) {
        std::cerr << "�޷�Ϊ���л������ҵ���Ч·��" << std::endl;
        return 1;
    }

    // ���·��
    planner.printPaths();

    // ���ӻ�·��
    planner.visualizePaths();

    // ��·�����浽�ļ�
    std::string output_filename = "paths_result.txt";
    if (!planner.savePathsToFile(output_filename)) {
        std::cerr << "����·�����ļ�ʧ��" << std::endl;
        return 1;
    }

    std::cout << "·���滮��ɣ�����ѱ��浽 " << output_filename << std::endl;

    return 0;
}