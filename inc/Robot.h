/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once

#include <memory>
#include <vector>
#include <deque>

// Core includes
#include <Utils.h>
#include <gbp/GBPCore.h>
#include <gbp/Factor.h>
#include <gbp/Factorgraph.h>

// Math library
#include <Eigen/Dense>

// Graphics
#include <raylib.h>

// Physics engines
#include <mujoco/mujoco.h>


// Forward declarations
class Simulator;
class Payload;

/***************************************************************************/
// Robot Class - Multi-robot collaborative manipulation with factor graphs
//
// Creates a robot with the following inputs:
//   - Pointer to the simulator
//   - Robot ID (should be taken from simulator->next_rid_++)
//   - Deque of waypoints (4-dimensional [x,y,xdot,ydot])
//   - Robot radius
//   - Color
//
// This is a derived class from the FactorGraph class
/***************************************************************************/
class Robot : public FactorGraph {
public:
    //==========================================================================
    // CONSTRUCTOR & DESTRUCTOR
    //==========================================================================
    Robot(Simulator* sim, 
            int rid, 
            std::deque<Eigen::VectorXd> waypoints, 
            float size, Color color);
    ~Robot();

    //==========================================================================
    // CORE ROBOT PROPERTIES
    //==========================================================================
    Simulator* sim_;                            // Pointer to the simulator
    int rid_ = 0;                               // Robot unique identifier
    std::deque<Eigen::VectorXd> waypoints_{};   // Waypoint queue [x,y,xdot,ydot]
    float robot_radius_ = 1.0f;                 // Robot physical radius
    Color color_ = DARKGREEN;                   // Robot visualization color
    Eigen::VectorXd position_;                  // Current position [x,y]
    float height_3D_ = 0.0f;                    // Height for 3D visualization
    int mujoco_body_id_;                        // MuJoCo body ID
    int mujoco_joint_id_;                       // MuJoCo joint ID (用于载荷连接)
    mjModel* mujoco_model_;                     // MuJoCo模型指针（从Simulator获取）
    mjData* mujoco_data_;                       // MuJoCo数据指针（从Simulator获取）

    //==========================================================================
    // FACTOR GRAPH STRUCTURE
    //==========================================================================
    int num_variables_;                         // Number of variables in planned path
    
    // Inter-robot communication and factors
    std::vector<int> connected_r_ids_{};        // Connected robot IDs via factors
    std::vector<int> neighbours_{};             // Robot IDs within communication range
    
    // Payload-related factor graph components
    std::vector<std::shared_ptr<Variable>> payload_twist_variables_;     // Payload twist variables
    std::vector<std::shared_ptr<PayloadTwistFactor>> payload_twist_factors_; // Corresponding factors
    std::vector<int> connected_payload_ids_;    // Connected payload IDs

    //==========================================================================
    // PHYSICS INTEGRATION
    //==========================================================================
    
    void syncToMuJoCo();                        // 同步逻辑状态到MuJoCo
    void syncFromMuJoCo();                      // 从MuJoCo同步状态
    void setMuJoCoReferences(mjModel* model, mjData* data); // 设置MuJoCo引用
    void createMuJoCoBody();                    // 创建MuJoCo body
    Eigen::Vector2d getMuJoCoPosition() const;
    Eigen::Vector2d getMuJoCoVelocity() const;
    double getMuJoCoRotation() const;
    double getMuJoCoAngularVelocity() const;
    double getMuJoCoMass() const;
    // 状态设置
    void setMuJoCoPosition(const Eigen::Vector2d& position);
    void setMuJoCoVelocity(const Eigen::Vector2d& velocity);
    void setMuJoCoRotation(double rotation);
    void setMuJoCoAngularVelocity(double angular_velocity);
    // 力控制
    void applyMuJoCoForce(const Eigen::Vector2d& force, const Eigen::Vector2d& point = Eigen::Vector2d::Zero());
    void applyMuJoCoTorque(double torque);

    //==========================================================================
    // PAYLOAD ATTACHMENT - MuJoCo约束
    //==========================================================================
    void attachToPayloadMuJoCo(std::shared_ptr<Payload> payload, const Eigen::Vector2d& attach_point);
    void detachFromPayloadMuJoCo();
    bool isAttachedToPayload() const { return mujoco_joint_id_ >= 0; }

    //==========================================================================
    // CONTACT & FORCE MANAGEMENT
    //==========================================================================
    // Contact point assignment
    int assigned_contact_point_index_;         // Assigned contact point index
    
    // Cached geometry parameters
    Eigen::Vector2d cached_r_vector_;          // Cached moment arm vector
    Eigen::Vector2d cached_contact_normal_;    // Cached contact normal vector
    bool payload_geometry_cached_;             // Whether geometry is cached
    
    // Force allocation
    std::vector<std::shared_ptr<Variable>> contact_force_variables_;  // Contact force variables
    std::shared_ptr<ForceAllocationFactor> force_allocation_factor_;  // Force allocation factor
    double assigned_contact_force_;            // Currently assigned contact force

    //==========================================================================
    // PAYLOAD INTERACTION METHODS
    //==========================================================================
    // Force-based methods
    void createContactForceVariables();
    void createForceAllocationFactor();
    void updateContactForceGeometry();
    double getAssignedContactForce() const { return assigned_contact_force_; }
    
    // Twist-based methods
    void updatePayloadTwistPriors();
    std::vector<Eigen::Vector3d> computeDesiredPayloadTwists();
    Eigen::Vector3d interpolatePayloadTwist(int var_index, 
                                           const Eigen::Vector3d& current_twist,
                                           const Eigen::Vector3d& target_twist);
    
    // Geometry management
    void updatePayloadFactorGeometry();
    void cachePayloadGeometry();
    
    // Physical attachment
    void attachToPayload(std::shared_ptr<Payload> payload, const Eigen::Vector2d& attach_point);
    void detachFromPayload();
    
    // Query methods
    bool isConnectedToPayload(int payload_id) const;
    int getAssignedContactPointIndex() const { return assigned_contact_point_index_; }

    //==========================================================================
    // FACTOR MANAGEMENT
    //==========================================================================
    void deletePayloadTwistFactors(int payload_id);
    
    // Inter-robot factors
    void updateInterrobotFactors();
    void createInterrobotFactors(std::shared_ptr<Robot> other_robot);
    void deleteInterrobotFactors(std::shared_ptr<Robot> other_robot);

    //==========================================================================
    // MOTION CONTROL & PHYSICS SYNC
    //==========================================================================
    // State updates
    void updateCurrent();                       // Update current state prior
    void updateHorizon();                      // Update horizon state prior
    
    void syncPhysicsToLogical();
    void syncLogicalToPhysics();
    void updateForPayload();
    
    // Velocity queries
    Eigen::Vector2d getCurrentVelocity() const;
    Eigen::Vector2d getDesiredVelocity() const;

    //==========================================================================
    // VISUALIZATION & DEBUGGING
    //==========================================================================
    Image* p_obstacleImage;                     // Obstacle environment image
    
    void draw();                                // Main drawing function
    void drawVelocityVector();                  // Draw velocity vectors for debugging

    //==========================================================================
    // UTILITY METHODS
    //==========================================================================
    // Variable timestep calculation
    std::vector<int> getVariableTimesteps(int H, int M);
    
    // Variable access operator
    std::shared_ptr<Variable>& operator[](const int& v_id) {
        int n = variables_.size();
        int search_vid = ((n + v_id) % n + n) % n;
        auto it = variables_.begin();
        std::advance(it, search_vid);
        return it->second;
    }

    //==========================================================================
    // HELPER METHODS (INLINE ACCESSORS)
    //==========================================================================
    std::shared_ptr<Variable> getVar(int index) const {
        if (index < 0 || index >= variables_.size()) return nullptr;
        auto it = variables_.begin();
        std::advance(it, index);
        return it->second;
    }
};