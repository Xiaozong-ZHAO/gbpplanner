/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include "Simulator.h"
#include <memory>
#include <vector>
#include <deque>
#include <raylib.h>
#include <Eigen/Dense>
#include <Utils.h>
#include <gbp/GBPCore.h>
#include <gbp/Factor.h>
#include <gbp/Factorgraph.h>
#include "box2d/box2d.h"

// extern Globals globals;

/***************************************************************************/
// Creates a robot. Inputs required are :
//      - Pointer to the simulator
//      - A robot id rid (should be taken from simulator->next_rid_++),
//      - A dequeue of waypoints (which are 4 dimensional [x,y,xdot,ydot])
//      - Robot radius
//      - Colour
// This is a derived class from the FactorGraph class
/***************************************************************************/
class Robot : public FactorGraph {
public:
    // Constructor
    Robot(Simulator* sim,
          int rid,
          std::deque<Eigen::VectorXd> waypoints,
          float size,
          Color color,
          b2World* world = nullptr);
    ~Robot();


    Simulator* sim_;                            // Pointer to the simulator
    int rid_ = 0;                               // Robot id
    std::deque<Eigen::VectorXd> waypoints_{};   // Dequeue of waypoints (whenever the robot reaches a point, it is popped off the front of the dequeue)
    float robot_radius_ = 1.;                   // Robot radius
    Color color_ = DARKGREEN;                   // Colour of robot

    int num_variables_;                         // Number of variables in the planned path (assumed to be the same for all robots)
    std::vector<int> connected_r_ids_{};        // List of robot ids that are currently connected via inter-robot factors to this robot
    std::vector<int> neighbours_{};             // List of robot ids that are within comms radius of this robot
    std::vector<int> nearby_{};
    Image* p_obstacleImage;                     // Pointer to image representing the obstacles in the environment
    float height_3D_ = 0.f;                     // Height out of plane (for 3d visualisation only)
    Eigen::VectorXd position_;                  // Position of the robot (equivalent to taking the [x,y] of the current state of the robot)
    
    std::vector<int> connected_payload_ids_;       // 类似connected_r_ids_
    int assigned_contact_point_index_;             // 分配的接触点索引（而不是坐标）
    b2Body* physicsBody_;
    b2World* physicsWorld_;
    bool usePhysics_;
    b2WeldJoint* payload_joint_;  // 新增：与payload的焊接关节

    // Payload factors removed - only dynamics, interrobot, and obstacle factors are used
    
    // 查询方法
    bool isConnectedToPayload(int payload_id) const;
    int getAssignedContactPointIndex() const { return assigned_contact_point_index_; }

    void attachToPayload(std::shared_ptr<Payload> payload, const Eigen::Vector2d& attach_point);
    void detachFromPayload();
    bool isAttachedToPayload() const { return payload_joint_ != nullptr; }
    /****************************************/
    //Functions
    /****************************************/
    void updateForPayload();
    void syncPhysicsToLogical();
    void syncLogicalToPhysics();
    void createPhysicsBody();
    /* Change the prior of the Current state */
    void updateCurrent();

    /* Change the prior of the Horizon state */    
    void updateHorizon();

    /***************************************************************************************************/
    // For new neighbours of a robot, create inter-robot factors if they don't exist. 
    // Delete existing inter-robot factors for faraway robots
    /***************************************************************************************************/    
    void updateInterrobotFactors();
    void createInterrobotFactors(std::shared_ptr<Robot> other_robot);
    void deleteInterrobotFactors(std::shared_ptr<Robot> other_robot);  
    

    /***************************************************************************************************/    
    // Drawing function
    /***************************************************************************************************/    
    void draw();

    /*******************************************************************************************/
    // Function for determining the timesteps at which variables in the planned path are placed.
    /*******************************************************************************************/   
    std::vector<int> getVariableTimesteps(int H, int M);


    /*******************************************************************************************/   
    // Access operator to get a pointer to a variable from the robot.
    /*******************************************************************************************/   
    std::shared_ptr<Variable>& operator[] (const int& v_id){
        int n = variables_.size();
        int search_vid = ((n + v_id) % n + n) % n;
        auto it = variables_.begin();
        std::advance(it, search_vid);
        return it->second;
    }    


};

