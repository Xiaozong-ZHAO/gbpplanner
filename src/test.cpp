#include "RobotGTSAM.h"
#include <Globals.h>
#include <gtsam/base/Vector.h>
#include <iostream>
#include <fstream>

// Declare globals instance (needed for GTSAM to access configuration)
Globals globals;

int main() {
    std::cout << "Running GTSAM test using RobotGTSAM class..." << std::endl;

    // Initialize globals from config file (matching GBP initialization)
    std::ifstream config_file("../config/config.json");
    if (config_file.is_open()) {
        globals.parse_global_args(config_file);
        globals.post_parsing();
        config_file.close();
        std::cout << "Configuration loaded: T_HORIZON=" << globals.T_HORIZON 
                  << ", LOOKAHEAD_MULTIPLE=" << globals.LOOKAHEAD_MULTIPLE << std::endl;
    } else {
        std::cerr << "Could not open config file, using defaults" << std::endl;
        // Set some defaults in case config fails
        globals.T_HORIZON = 10.0;
        globals.T0 = 0.1;
        globals.LOOKAHEAD_MULTIPLE = 3;
        globals.SIGMA_FACTOR_DYNAMICS = 0.01;
        globals.SIGMA_POSE_FIXED = 1e-15;
    }

    // Define start and target states [x, y, xdot, ydot]
    gtsam::Vector4 start_state;
    start_state << 0.0, 0.0, 0.0, 0.0;  // Starting at origin with zero velocity

    gtsam::Vector4 target_state;
    target_state << 10.0, 5.0, 1.0, 0.5;  // Target at (10,5) with velocity (1,0.5)

    // Create robot with configuration-driven factor graph (like GBP)
    RobotGTSAM robot(start_state, target_state);

    // Optimize the trajectory
    robot.optimize();

    std::cout << "GTSAM test completed." << std::endl;
    return 0;
}
