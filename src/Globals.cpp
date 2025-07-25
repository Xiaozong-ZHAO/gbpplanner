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
    OBSTACLE_CONFIG_FILE = j["OBSTACLE_CONFIG_FILE"];

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

    SIGMA_FACTOR_CONTACT = j["SIGMA_FACTOR_CONTACT"];
    SIGMA_FACTOR_PAYLOAD_VELOCITY = j["SIGMA_FACTOR_PAYLOAD_VELOCITY"];
    SIGMA_FACTOR_GEOMETRY = j["SIGMA_FACTOR_GEOMETRY"];
    USE_DISTRIBUTED_PAYLOAD_CONTROL = static_cast<bool>((int)j["USE_DISTRIBUTED_PAYLOAD_CONTROL"]);
    CONTACT_ASSIGNMENT_RADIUS = j["CONTACT_ASSIGNMENT_RADIUS"];
    OBSTACLE_PADDING = j["OBSTACLE_PADDING"];

    STARTING_X = j["STARTING_X"];
    STARTING_Y = j["STARTING_Y"];
    TARGET_RELATIVE_X = j["TARGET_RELATIVE_X"];
    TARGET_RELATIVE_Y = j["TARGET_RELATIVE_Y"];
    TARGET_RELATIVE_ROTATION = j["TARGET_RELATIVE_ROTATION"];
    USE_RIGID_ATTACHMENT = static_cast<bool>((int)j["USE_RIGID_ATTACHMENT"]);
    USE_DIRECT_PAYLOAD_VELOCITY = static_cast<bool>((int)j["USE_DIRECT_PAYLOAD_VELOCITY"]);  // 新增

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
