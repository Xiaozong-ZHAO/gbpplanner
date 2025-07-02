/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <Globals.h>
#include <Utils.h>
#include "json.hpp"

/*****************************************************************/
// Simply reads the appropriate sections from the config.json
/*****************************************************************/
void Globals::parse_global_args(std::ifstream& config_file){
    
    // Basic parameters
    nlohmann::json j;
    config_file >> j;
    ASSETS_DIR = j["ASSETS_DIR"];

    // Display parameters
    DISPLAY = static_cast<bool>((int)j["DISPLAY"]);;
    WORLD_SZ = j["WORLD_SZ"];
    SCREEN_SZ = j["SCREEN_SZ"];
    DRAW_INTERROBOT = static_cast<bool>((int)j["DRAW_INTERROBOT"]);
    DRAW_PATH = static_cast<bool>((int)j["DRAW_PATH"]);
    DRAW_WAYPOINTS = static_cast<bool>((int)j["DRAW_WAYPOINTS"]);

    // Simulation parameters
    SEED = j["SEED"];
    TIMESTEP = j["TIMESTEP"];
    MAX_TIME = j["MAX_TIME"];
    NUM_ROBOTS = j["NUM_ROBOTS"];
    T_HORIZON = j["T_HORIZON"];
    ROBOT_RADIUS = j["ROBOT_RADIUS"];
    COMMUNICATION_RADIUS = j["COMMUNICATION_RADIUS"];
    MAX_SPEED = j["MAX_SPEED"];
    COMMS_FAILURE_RATE = j["COMMS_FAILURE_RATE"];
    FORMATION = j["FORMATION"];
    OBSTACLE_FILE = j["OBSTACLE_FILE"];

    // GBP parameters
    SIGMA_FACTOR_DYNAMICS = j["SIGMA_FACTOR_DYNAMICS"];
    SIGMA_FACTOR_INTERROBOT = j["SIGMA_FACTOR_INTERROBOT"];
    SIGMA_FACTOR_OBSTACLE = j["SIGMA_FACTOR_OBSTACLE"];
    NUM_ITERS = j["NUM_ITERS"];

    // Payload parameters
    PAYLOAD_ENABLED = static_cast<bool>((int)j["PAYLOAD_ENABLED"]);
    PAYLOAD_WIDTH = j["PAYLOAD_WIDTH"];
    PAYLOAD_HEIGHT = j["PAYLOAD_HEIGHT"];
    PAYLOAD_DENSITY = j["PAYLOAD_DENSITY"];
    PAYLOAD_FRICTION = j["PAYLOAD_FRICTION"];
    TARGET_X = j["TARGET_X"];
    TARGET_Y = j["TARGET_Y"];
    MAX_ANGULAR_SPEED = j["MAX_ANGULAR_SPEED"];

    // SIGMA_FACTOR_CONTACT = j["SIGMA_FACTOR_CONTACT"];
    // SIGMA_FACTOR_PAYLOAD_VELOCITY = j["SIGMA_FACTOR_PAYLOAD_VELOCITY"];
    USE_DISTRIBUTED_PAYLOAD_CONTROL = static_cast<bool>((int)j["USE_DISTRIBUTED_PAYLOAD_CONTROL"]);
    CONTACT_ASSIGNMENT_RADIUS = j["CONTACT_ASSIGNMENT_RADIUS"];

    TARGET_RELATIVE_X = j["TARGET_RELATIVE_X"];
    TARGET_RELATIVE_Y = j["TARGET_RELATIVE_Y"];
    TARGET_RELATIVE_ROTATION = j["TARGET_RELATIVE_ROTATION"];
    USE_RIGID_ATTACHMENT = static_cast<bool>((int)j["USE_RIGID_ATTACHMENT"]);
    USE_DIRECT_PAYLOAD_VELOCITY = static_cast<bool>((int)j["USE_DIRECT_PAYLOAD_VELOCITY"]);  // 新增
    DRAW_ROBOT_VELOCITIES = static_cast<bool>((int)j["DRAW_ROBOT_VELOCITIES"]);
    SIGMA_FACTOR_PAYLOAD_TWIST = j["SIGMA_FACTOR_PAYLOAD_TWIST"];
    SIGMA_FACTOR_FORCE_ALLOCATION = j.value("SIGMA_FACTOR_FORCE_ALLOCATION", 1.0f);
    USE_FORCE_ALLOCATION = static_cast<bool>((int)j.value("USE_FORCE_ALLOCATION", 1));

    USE_MUJOCO_PHYSICS = static_cast<bool>((int)j.value("USE_MUJOCO_PHYSICS", 0));
    MUJOCO_MODEL_FILE = j.value("MUJOCO_MODEL_FILE", "");
    MUJOCO_TIMESTEP = j.value("MUJOCO_TIMESTEP", 0.001f);
    MUJOCO_SOLVER_ITERATIONS = j.value("MUJOCO_SOLVER_ITERATIONS", 50);
    MUJOCO_CONTACT_DAMPING = j.value("MUJOCO_CONTACT_DAMPING", 100.0f);
    MUJOCO_ENABLE_VISUALIZATION = static_cast<bool>((int)j.value("MUJOCO_ENABLE_VISUALIZATION", 1));
    MUJOCO_VISUALIZATION_WIDTH = j.value("MUJOCO_VISUALIZATION_WIDTH", 800);
    MUJOCO_VISUALIZATION_HEIGHT = j.value("MUJOCO_VISUALIZATION_HEIGHT", 600);
    
    // 重力设置
    if (j.contains("MUJOCO_GRAVITY") && j["MUJOCO_GRAVITY"].is_array()) {
        MUJOCO_GRAVITY.clear();
        for (auto& gravity_component : j["MUJOCO_GRAVITY"]) {
            MUJOCO_GRAVITY.push_back(gravity_component);
        }
    } else {
        MUJOCO_GRAVITY = {0.0f, 0.0f, 0.0f};  // 默认零重力
    }
    
    // 高级MuJoCo设置
    MUJOCO_INTEGRATOR = j.value("MUJOCO_INTEGRATOR", "RK4");
    MUJOCO_CONE = j.value("MUJOCO_CONE", "pyramidal");
    MUJOCO_JACOBIAN = j.value("MUJOCO_JACOBIAN", "sparse");
    MUJOCO_SOLVER = j.value("MUJOCO_SOLVER", "PGS");
    MUJOCO_ITERATIONS = j.value("MUJOCO_ITERATIONS", 100);
    MUJOCO_TOLERANCE = j.value("MUJOCO_TOLERANCE", 1e-8f);
    MUJOCO_LS_ITERATIONS = j.value("MUJOCO_LS_ITERATIONS", 50);
    MUJOCO_LS_TOLERANCE = j.value("MUJOCO_LS_TOLERANCE", 0.01f);
    
    // 接触模型参数
    MUJOCO_CONTACT_MARGIN = j.value("MUJOCO_CONTACT_MARGIN", 0.001f);
    MUJOCO_CONTACT_GAP = j.value("MUJOCO_CONTACT_GAP", 0.0f);
    MUJOCO_CONTACT_FRICTION_LOSS = j.value("MUJOCO_CONTACT_FRICTION_LOSS", 0.01f);
    
    // 解析SOLREF参数 [timeconst, dampratio]
    if (j.contains("MUJOCO_CONTACT_SOLREF") && j["MUJOCO_CONTACT_SOLREF"].is_array()) {
        MUJOCO_CONTACT_SOLREF.clear();
        for (auto& param : j["MUJOCO_CONTACT_SOLREF"]) {
            MUJOCO_CONTACT_SOLREF.push_back(param);
        }
    } else {
        MUJOCO_CONTACT_SOLREF = {0.02f, 1.0f};  // 默认值
    }
    
    // 解析SOLIMP参数 [dmin, dmax, width, midpoint, power]
    if (j.contains("MUJOCO_CONTACT_SOLIMP") && j["MUJOCO_CONTACT_SOLIMP"].is_array()) {
        MUJOCO_CONTACT_SOLIMP.clear();
        for (auto& param : j["MUJOCO_CONTACT_SOLIMP"]) {
            MUJOCO_CONTACT_SOLIMP.push_back(param);
        }
    } else {
        MUJOCO_CONTACT_SOLIMP = {0.9f, 0.95f, 0.001f, 0.5f, 2.0f};  // 默认值
    }
    
    // 调试设置
    ENABLE_PHYSICS_LOGGING = static_cast<bool>((int)j.value("ENABLE_PHYSICS_LOGGING", 0));
    LOG_CONTACT_FORCES = static_cast<bool>((int)j.value("LOG_CONTACT_FORCES", 0));
    LOG_JOINT_CONSTRAINTS = static_cast<bool>((int)j.value("LOG_JOINT_CONSTRAINTS", 0));
    PHYSICS_DEBUG_VISUALIZATION = static_cast<bool>((int)j.value("PHYSICS_DEBUG_VISUALIZATION", 0));
    // STIFFNESS = j.value("MUJOCO_CONTACT_STIFFNESS", 10000.0f);
    // MUJOCO_CONTACT_

}

Globals::Globals(){};

/*****************************************************************/
// Allows for parsing of an external config file
/*****************************************************************/
int Globals::parse_global_args(DArgs::DArgs &dargs)
{
    // Argument parser
    this->CONFIG_FILE = dargs("--cfg", "config_file", this->CONFIG_FILE);
    
    if (!dargs.check())
    {
        dargs.print_help();
        print("Incorrect arguments!");
        return EXIT_FAILURE;
    }

    std::ifstream my_config_file(CONFIG_FILE);
    assert(my_config_file && "Couldn't find the config file");
    parse_global_args(my_config_file);
    post_parsing();

    return 0;
};

/*****************************************************************/
// Any checks on the input configs should go here.
/*****************************************************************/
void Globals::post_parsing()
{
    // Cap max speed, since it should be <= ROBOT_RADIUS/2.f / TIMESTEP:
    // In one timestep a robot should not move more than half of its radius
    // (since we plan for discrete timesteps)
    if (MAX_SPEED > ROBOT_RADIUS/2.f/TIMESTEP){
        MAX_SPEED = ROBOT_RADIUS/2.f/TIMESTEP;
        print("Capping MAX_SPEED parameter at ", MAX_SPEED);
    }
    T0 = ROBOT_RADIUS/2.f / MAX_SPEED; // Time between current state and next state of planned path

}
