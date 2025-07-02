/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <cmath>
#include <raylib.h>
#include <DArgs.h>
#include <fstream>
#include "json.hpp"

// Simulation modes
enum MODES_LIST {SimNone, Timestep, Iterate, Help};

/***********************************************************************************************/
// Global structure. The values here are mostly set using the provided config.json file.
/***********************************************************************************************/
class Globals {
    public:
    // float SIGMA_FACTOR_CONTACT;           // 接触保持因子强度
    // float SIGMA_FACTOR_PAYLOAD_VELOCITY;  // 速度对齐因子强度
    bool USE_DISTRIBUTED_PAYLOAD_CONTROL; // 是否使用分布式控制
    float CONTACT_ASSIGNMENT_RADIUS;      // 接触点分配半径

    // Basic parameters
    const char* WINDOW_TITLE = "Distributing Multirobot Motion Planning with Gaussian Belief Propogation";
    bool RUN = true;
    std::string CONFIG_FILE = "../config/config.json";      // Default config file
    std::string OBSTACLE_FILE;                              // Binary image for obstacles
    std::string ASSETS_DIR;                                 // Directory for Assets
    int N_DOFS = 4;                                         // Degrees of freedom (x, y, xdot, ydot)
    MODES_LIST SIM_MODE = Timestep;                         // Simulation mode to begin with
    MODES_LIST LAST_SIM_MODE = Timestep;                    // Storage of Simulation mode (if time is paused eg.)
    
    // Display parameters
    bool DISPLAY;                                           // Show display or not
    int WORLD_SZ;                                           // [m]
    int SCREEN_SZ;                                          // [pixels]
    bool DRAW_INTERROBOT;                                   // Toggle display of inter-robot connections
    bool DRAW_PATH;                                         // Toggle display of planned paths
    bool DRAW_WAYPOINTS;                                    // Toggle display of path planning goals
    
    // Simulation parameters
    int SEED;                                               // Random Seed 
    float TIMESTEP;                                         // Simulation timestep [s]
    int MAX_TIME;                                           // Exit simulation if more timesteps than this
    int NUM_ROBOTS;                                         // Number of robots (if no new robots are to be added)
    float T_HORIZON;                                        // Planning horizon [s]
    float ROBOT_RADIUS;                                     // [m]
    float COMMUNICATION_RADIUS;                             // [m] Inter-robot factors created if robots are within this range of each other
    float MAX_SPEED;                                        // [m/s]
    float COMMS_FAILURE_RATE;                               // Proportion of robots [0,1] that do not communicate
    int LOOKAHEAD_MULTIPLE = 3;                             // Parameter affecting how planned path is spaced out in time
    std::string FORMATION;                                    // Robot formation (CIRCLE or JUNCTION)
    float T0;                                               // Time between current state and next state of planned path

    // GBP parameters
    float SIGMA_POSE_FIXED = 1e-15;                         // Sigma for Unary pose factor on current and horizon states
    float SIGMA_FACTOR_DYNAMICS;                            // Sigma for Dynamics factors
    float SIGMA_FACTOR_INTERROBOT;                          // Sigma for Interrobot factor
    float SIGMA_FACTOR_OBSTACLE;                            // Sigma for Static obstacle factors
    int NUM_ITERS;                                          // Number of iterations of GBP per timestep
    float DAMPING = 0.;                                     // Damping amount (not used in this work)
    
    // Payload parameters
    bool PAYLOAD_ENABLED;
    float PAYLOAD_WIDTH;
    float PAYLOAD_HEIGHT;
    float PAYLOAD_DENSITY;
    float PAYLOAD_FRICTION;
    float TARGET_X;
    float TARGET_Y;
    float MAX_ANGULAR_SPEED;

    float TARGET_RELATIVE_X;
    float TARGET_RELATIVE_Y;
    float TARGET_RELATIVE_ROTATION;
    bool USE_RIGID_ATTACHMENT;
    bool USE_DIRECT_PAYLOAD_VELOCITY; // 是否使用直接速度控制
    bool DRAW_ROBOT_VELOCITIES;
    float SIGMA_FACTOR_PAYLOAD_TWIST;
    float SIGMA_FACTOR_FORCE_ALLOCATION;
    bool USE_FORCE_ALLOCATION;
    
    Globals();
    int parse_global_args(DArgs::DArgs& dargs);
    void parse_global_args(std::ifstream& config_file);
    void post_parsing();

    bool USE_MUJOCO_PHYSICS;
    std::string MUJOCO_MODEL_FILE;
    float MUJOCO_TIMESTEP;
    int MUJOCO_SOLVER_ITERATIONS;
    float MUJOCO_CONTACT_STIFFNESS;
    float MUJOCO_CONTACT_DAMPING;
    std::vector<float> MUJOCO_GRAVITY;
    bool MUJOCO_ENABLE_VISUALIZATION;
    int MUJOCO_VISUALIZATION_WIDTH;
    int MUJOCO_VISUALIZATION_HEIGHT;
    
    // 高级MuJoCo设置
    std::string MUJOCO_INTEGRATOR;
    std::string MUJOCO_CONE;
    std::string MUJOCO_JACOBIAN;
    std::string MUJOCO_SOLVER;
    int MUJOCO_ITERATIONS;
    float MUJOCO_TOLERANCE;
    int MUJOCO_LS_ITERATIONS;
    float MUJOCO_LS_TOLERANCE;
    
    // 接触模型参数
    float MUJOCO_CONTACT_MARGIN;
    float MUJOCO_CONTACT_GAP;
    float MUJOCO_CONTACT_FRICTION_LOSS;
    std::vector<float> MUJOCO_CONTACT_SOLREF;
    std::vector<float> MUJOCO_CONTACT_SOLIMP;
    
    // 调试设置
    bool ENABLE_PHYSICS_LOGGING;
    bool LOG_CONTACT_FORCES;
    bool LOG_JOINT_CONSTRAINTS;
    bool PHYSICS_DEBUG_VISUALIZATION;


};

extern Globals globals;