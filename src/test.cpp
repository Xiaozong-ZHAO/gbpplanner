#include "RobotGTSAM.h"
#include <gtsam/base/Vector.h>
#include <iostream>

int main() {
    std::cout << "Running GTSAM test using RobotGTSAM class..." << std::endl;

    RobotGTSAM robot;

    // 添加两个测试状态 [x, y, xdot, ydot]
    gtsam::Vector4 state1;
    state1 << 1.0, 0.0, 0.5, 0.0;  // x=1, y=0, xdot=0.5, ydot=0

    gtsam::Vector4 state2;
    state2 << 2.0, 0.2, 0.4, 0.1;  // x=2, y=0.2, xdot=0.4, ydot=0.1

    robot.addState(state1);
    robot.addState(state2);

    robot.optimize();

    std::cout << "GTSAM test completed." << std::endl;
    return 0;
}
