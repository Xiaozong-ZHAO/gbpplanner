/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#ifndef PAYLOAD_H
#define PAYLOAD_H

// Core includes
#include <memory>

// Math library
#include <Eigen/Dense>

// Graphics
#include <raylib.h>

// Physics engines
#include <mujoco/mujoco.h>


// Forward declarations
class Simulator;

/*****************************************************************************************/
// PAYLOAD CLASS - Represents objects being manipulated by multiple robots
//
// The Payload class manages:
// - Physical properties (mass, inertia, dimensions)
// - Spatial state (position, orientation, velocity)
// - Target tracking (desired position and orientation)
// - Contact interface for robot interaction
// - Physics integration (Box2D and optional MuJoCo)
/*****************************************************************************************/
class Payload {
public:
    //==========================================================================
    // CONSTRUCTOR & DESTRUCTOR
    //==========================================================================
    Payload(Simulator* sim,
            int payload_id,
            Eigen::Vector2d initial_position,
            float width,
            float height,
            float density,
            Color color = GRAY);
    ~Payload();

    //==========================================================================
    // CORE IDENTIFICATION
    //==========================================================================
    int payload_id_;                            // Unique payload identifier
    Simulator* sim_;                            // Pointer to simulator

    //==========================================================================
    // PHYSICAL PROPERTIES
    //==========================================================================
    // Geometric properties
    float width_, height_;                      // Payload dimensions
    Color color_;                               // Visualization color
    
    // Dynamic properties
    double mass_;                               // Payload mass
    double moment_of_inertia_;                  // Rotational inertia
    double current_angular_velocity_;           // Current angular velocity

    //==========================================================================
    // SPATIAL STATE
    //==========================================================================
    // Position and velocity
    Eigen::Vector2d position_;                  // Current position [x, y]
    Eigen::Vector2d velocity_;                  // Current linear velocity
    float rotation_;                            // Current rotation angle (radians)
    
    // Orientation (quaternion representation)
    Eigen::Quaterniond initial_orientation_ = Eigen::Quaterniond::Identity();   // Initial orientation
    Eigen::Quaterniond current_orientation_ = Eigen::Quaterniond::Identity();   // Current orientation
    Eigen::Quaterniond target_orientation_ = Eigen::Quaterniond::Identity();    // Target orientation

    //==========================================================================
    // TARGET & TASK MANAGEMENT
    //==========================================================================
    Eigen::Vector2d target_position_;          // Desired final position
    bool task_completed_;                       // Task completion flag
    float target_tolerance_;                    // Distance tolerance for target reaching

    //==========================================================================
    // PHYSICS INTEGRATION
    //==========================================================================



    int mujoco_body_id_;                        // MuJoCo body identifier
    mjModel* mujoco_model_;
    mjData* mujoco_data_;

    
    void syncToMuJoCo();
    void syncFromMuJoCo();
    void setMuJoCoReferences(mjModel* model, mjData* data);

    //==========================================================================
    // PHYSICS BODY MANAGEMENT
    //==========================================================================
    void createMuJoCoBody(float density);

    Eigen::Vector2d getMuJoCoPosition() const;
    Eigen::Vector2d getMuJoCoVelocity() const;
    double getMuJoCoRotation() const;
    double getMuJoCoAngularVelocity() const;
    double getMuJoCoMass() const;
    double getMuJoCoInertia() const;

    // 状态设置
    void setMuJoCoPosition(const Eigen::Vector2d& position);
    void setMuJoCoVelocity(const Eigen::Vector2d& velocity);
    void setMuJoCoRotation(double rotation);
    void setMuJoCoAngularVelocity(double angular_velocity);
    void applyMuJoCoForce(const Eigen::Vector2d& force, const Eigen::Vector2d& point = Eigen::Vector2d::Zero());
    void applyMuJoCoTorque(double torque);

    //==========================================================================
    // CONTACT INTERFACE
    //==========================================================================
    // Contact geometry for robot interaction
    std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> 
        getContactPointsAndNormals() const;    // Get contact points and normals

    //==========================================================================
    // TARGET CONTROL
    //==========================================================================
    // Position targets
    void setTarget(const Eigen::Vector2d& target);
    void setTarget(const Eigen::Vector2d& target_position, const Eigen::Quaterniond& target_orientation);
    
    // Orientation targets
    void setTargetFromRelativeRotation(double relative_rotation_rad);
    void updateTargetOrientation();

    //==========================================================================
    // STATE QUERIES
    //==========================================================================
    // Position and orientation
    Eigen::Vector2d getPosition() const;
    Eigen::Quaterniond getRotation() const;
    Eigen::Vector2d getVelocity() const;
    double getAngularVelocity() const;
    
    // Physical properties
    float getMass() const;
    double getMomentOfInertia() const;
    
    // Target tracking
    bool isAtTarget() const;
    Eigen::Vector2d getDistanceToTarget() const;
    float getDistanceToTargetMagnitude() const;
    double getRotationError() const;
    
    // Control assistance
    Eigen::Vector2d getRequiredPushDirection() const;
    bool shouldStopPushing() const;

    //==========================================================================
    // SIMULATION INTERFACE
    //==========================================================================
    void update();                              // Update payload state
    void draw();                                // Render payload visualization

    //==========================================================================
    // UTILITY METHODS
    //==========================================================================
    double getRotationFromQuaternion(const Eigen::Quaterniond& q) const;

private:
    //==========================================================================
    // PRIVATE MEMBERS
    //==========================================================================
    // Additional private state or helper methods can be added here
};

#endif // PAYLOAD_H