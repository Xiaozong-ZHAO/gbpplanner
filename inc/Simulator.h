/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once

#include <map>
#include <memory>
#include <algorithm>
#include <fstream>

#include <Utils.h>
#include <gbp/GBPCore.h>
#include <Graphics.h>
#include <gbp/Variable.h>
#include <nanoflann.h>

#include <raylib.h>
#include <rlights.h>
#include <nanoflann.h>
#include <KDTreeMapOfVectorsAdaptor.h>
#include <random>
#include "box2d/box2d.h"
#include "Payload.h"

class Robot;
class RobotGTSAM;
class Graphics;
class TreeOfRobots;
class Payload;

// Simple obstacle structure
struct ObstacleData {
    int id;
    Eigen::Vector2d position;
    float radius;
};

/************************************************************************************/
// The main Simulator. This is where the magic happens.
/************************************************************************************/
class Simulator {
public:
    friend class Robot;
    friend class Factor;
    friend class Payload;

    // Constructor
    Simulator();
    ~Simulator();

    // Pointer to Graphics class which hold all the camera, graphics and models for display
    Graphics* graphics;

    // kd-tree to store the positions of the robots at each timestep.
    // This is used for calculating the neighbours of robots blazingly fast.
    typedef KDTreeMapOfVectorsAdaptor<std::map<int,std::vector<double>>> KDTree;
    std::map<int, std::vector<double>> robot_positions_{{0,{0.,0.}}};
    KDTree* treeOfRobots_;

    // Image representing the obstacles in the environment
    std::vector<Eigen::Vector2d> endings_;
    Image obstacleImg;
    
    // Obstacle data loaded from JSON
    std::vector<ObstacleData> obstacles_;

    int next_rid_ = 0;                              // New robots will use this rid. It should be ++ incremented when this happens
    int next_vid_ = 0;                              // New variables will use this vid. It should be ++ incremented when this happens
    int next_fid_ = 0;                              // New factors will use this fid. It should be ++ incremented when this happens
    int next_payload_id_ = 0;
    uint32_t clock_ = 0;                            // Simulation clock (timesteps)                   
    std::map<int, std::shared_ptr<Robot>> robots_;  // Map containing smart pointers to all robots, accessed by their rid.
    std::map<int, std::shared_ptr<RobotGTSAM>> robots_gtsam_; // Map containing smart pointers to all GTSAM robots, accessed by their rid.
    std::map<int, std::shared_ptr<Payload>> payloads_; // Map containing smart pointers to all payloads, accessed by their pid.
    bool new_robots_needed_ = true;                 // Whether or not to create new robots. (Some formations are dynamicaly changing)
    bool symmetric_factors = false;                 // If true, when inter-robot factors need to be created between two robots,
                                                    // a pair of factors is created (one belonging to each robot). This becomes a redundancy.
    std::pair<Eigen::Vector2d, Eigen::Vector2d> getContactPoint(int robot_id, int payload_id);
    bool isRobotContactingPayload(int robot_id, int payload_id);
    b2World* physicsWorld_ = nullptr;
    std::ofstream trajectory_csv_file_;                     // CSV file stream for trajectory export
    double gbp_duration_microseconds_ = 0.0;               // Last GBP iteration duration in microseconds


    b2World* getPhysicsWorld(){
        if (physicsWorld_ == nullptr){
            b2Vec2 gravity(0.0f, 0.0f); // No gravity
            physicsWorld_ = new b2World(gravity);
        }
        return physicsWorld_;
    }
    /*******************************************************************************/
    // Create new robots if needed. Handles deletion of robots out of bounds. 
    // New formations must modify the vectors "robots to create" and optionally "robots_to_delete"
    // by appending (push_back()) a shared pointer to a Robot class.
    /*******************************************************************************/    
    std::map<int, std::shared_ptr<Payload>> getPayload();
    void assignContactPoints();
    void updateDistributedPayloadControl();
    // 辅助方法
    std::vector<int> getNearbyRobots(int payload_id, double radius);
    void reassignLostContacts(int payload_id);

    void createOrDeleteRobots();
    void createOrDeleteRobotsGTSAM();
    void timestepGTSAM();
    void syncGTSAMPhysicsToLogical();
    void syncGTSAMLogicalToPhysics();
    void updateDistributedPayloadControlGTSAM();
    void optimizeAllGTSAMRobots();
    void updateGTSAMRobotStates();
    void isRobotContactngPayload(int robot_id, int payload_id);
    void createPayload(Eigen::Vector2d position, float width, float height);
    void deletePayload(int payload_id);
    std::vector<Eigen::Vector2d> getFixedContactPoints(std::shared_ptr<Payload> payload);
    std::vector<Eigen::Vector2d> getFixedContactNormals(std::shared_ptr<Payload> payload);
    std::shared_ptr<Payload> getPayload(int payload_id);
    Eigen::MatrixXd Quat2Rot(Eigen::Quaterniond q);
    
    // Obstacle management functions
    void loadObstacles();
    void drawObstacles();

    // CSV export functions
    void initCSVExport();
    void exportPayloadTrajectory();

    /*******************************************************************************/
    // Set a proportion of robots to not perform inter-robot communications
    /*******************************************************************************/
    void setCommsFailure(float failure_rate=globals.COMMS_FAILURE_RATE);

    /*******************************************************************************/
    // Timestep loop of simulator.
    /*******************************************************************************/
    void timestep();

    /*******************************************************************************/
    // Drawing graphics.
    /*******************************************************************************/
    void draw();

    void draw_payloads();

    /*******************************************************************************/
    // Use a kd-tree to perform a radius search for neighbours of a robot within comms. range
    // (Updates the neighbours_ of a robot)
    /*******************************************************************************/    
    void calculateRobotNeighbours(std::map<int,std::shared_ptr<Robot>>& robots);
    void locateNearbys(std::map<int, std::shared_ptr<Robot>>& robots);

    /*******************************************************************************/
    // Handles keypresses and mouse input, and updates camera.
    /*******************************************************************************/
    void eventHandler();

    /*******************************************************************************/
    // Deletes the robot from the simulator's robots_, as well as any variable/factors associated.
    /*******************************************************************************/
    void deleteRobot(std::shared_ptr<Robot> robot);

    /***************************************************************************************************************/
    // RANDOM NUMBER GENERATOR.
    // Usage: random_number("normal", mean, sigma) or random_number("uniform", lower, upper)
    /***************************************************************************************************************/
    std::mt19937 gen_normal = std::mt19937(globals.SEED);
    std::mt19937 gen_uniform = std::mt19937(globals.SEED);
    std::mt19937 gen_uniform_int = std::mt19937(globals.SEED);
    template<typename T>
    T random_number(std::string distribution, T param1, T param2){
        if (distribution=="normal") return std::normal_distribution<T>(param1, param2)(gen_normal);
        if (distribution=="uniform") return std::uniform_real_distribution<T>(param1, param2)(gen_uniform);
        return (T)0;
    }
    int random_int(int lower, int upper){
        return std::uniform_int_distribution<int>(lower, upper)(gen_uniform_int);
    }

};
