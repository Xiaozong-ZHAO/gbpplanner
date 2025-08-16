#include "RobotGTSAM.h"
#include <gtsam/base/Vector.h>
#include <iostream>

int main() {
    std::cout << "Running GTSAM test using RobotGTSAM class..." << std::endl;

    // Define start and target states [x, y, xdot, ydot]
    gtsam::Vector4 start_state;
    start_state << 0.0, 0.0, 0.0, 0.0;  // Starting at origin with zero velocity

    gtsam::Vector4 target_state;
    target_state << 10.0, 5.0, 1.0, 0.5;  // Target at (10,5) with velocity (1,0.5)

    // Create robot with complete factor graph (24 variables like GBP)
    RobotGTSAM robot(start_state, target_state, 24, 0.1);

    // Optimize the trajectory
    robot.optimize();

    std::cout << "GTSAM test completed." << std::endl;
    return 0;
}
