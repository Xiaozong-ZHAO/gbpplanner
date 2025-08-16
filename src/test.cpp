#define RLIGHTS_IMPLEMENTATION // needed to be defined once for the lights shader
#include "RobotGTSAM.h"
#include <Globals.h>
#include <Simulator.h>
#include <gtsam/base/Vector.h>
#include <iostream>
#include <fstream>

// Declare globals instance (needed for GTSAM to access configuration)
Globals globals;

int main() {
    std::cout << "Running GTSAM test with visualization..." << std::endl;

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
        globals.DISPLAY = 1;
        globals.SCREEN_SZ = 800;
        globals.WORLD_SZ = 200;
        globals.DRAW_PATH = 1;
        globals.DRAW_WAYPOINTS = 1;
        globals.OBSTACLE_FILE = "";
    }

    // Create minimal simulator for graphics support
    Simulator* sim = new Simulator();

    // Define start and target states [x, y, xdot, ydot]
    gtsam::Vector4 start_state;
    start_state << 0.0, 0.0, 0.0, 0.0;  // Starting at origin with zero velocity

    gtsam::Vector4 target_state;
    target_state << 10.0, 5.0, 1.0, 0.5;  // Target at (10,5) with velocity (1,0.5)

    // Create robot with visualization support
    RobotGTSAM robot(start_state, target_state, sim, BLUE, 1.0f);

    // Optimize the trajectory
    robot.optimize();

    // Simple visualization test - render one frame
    if (globals.DISPLAY && sim->graphics) {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        
        // Set up 3D mode with simple camera
        Camera3D camera = { 0 };
        camera.position = (Vector3){ 15.0f, 15.0f, 15.0f };
        camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
        camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
        camera.fovy = 45.0f;
        camera.projection = CAMERA_PERSPECTIVE;
        
        BeginMode3D(camera);
        
        // Draw the robot with its optimized path
        robot.draw();
        
        // Draw ground plane for reference
        DrawPlane((Vector3){ 0.0f, 0.0f, 0.0f }, (Vector2){ 50.0f, 50.0f }, LIGHTGRAY);
        
        EndMode3D();
        
        DrawText("GTSAM Robot Trajectory Visualization", 10, 10, 20, DARKGRAY);
        DrawText("Robot optimized path shown in blue", 10, 40, 16, DARKGRAY);
        
        EndDrawing();
        
        // Keep window open for a moment to see the result
        std::cout << "Visualization rendered. Press any key to continue..." << std::endl;
        while (!WindowShouldClose() && !IsKeyPressed(KEY_SPACE)) {
            // Wait for user input or window close
        }
    }

    delete sim;
    CloseWindow();

    std::cout << "GTSAM test completed." << std::endl;
    return 0;
}
