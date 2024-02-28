#include <liborl/liborl.h>
#include <memory>
#include <queue>
#include <array>

int main(int argc, char **argv) {
    std::queue<std::array<double, 3>> q;
    std::array<double, 3> a1 = {0.7, 0, 0};
    std::array<double, 3> a2 = {0.6, 0.2, 0};
    std::array<double, 3> a3 = {0.6,-0.2, 0};
    std::array<double, 3> a4 = {0.4, 0.3, 0};
    std::array<double, 3> a5 = {0.4,-0.3, 0};
    q.push(a1);
    q.push(a2);
    q.push(a3);
    q.push(a4);
    q.push(a5);
    orl::Robot robot("172.16.0.2");
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    robot.joint_motion(q_goal, 0.2);
    robot.absolute_cart_motion(0.336, 0, -0.07, 3);
    std::cout << "Reached init point" << std::endl;
    std::cin.ignore();
    int idx = 0;
    while (!q.empty()) {
        idx++;
        robot.absolute_cart_motion(q.front()[0], q.front()[1], q.front()[2], 3);
        std::cout << "Reached point " << idx << std::endl;
        std::cin.ignore();
        q.pop();
    }
}

