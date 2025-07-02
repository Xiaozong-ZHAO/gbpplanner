/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once

// Core includes
#include <Utils.h>
#include <gbp/GBPCore.h>

// Math library
#include <Eigen/Dense>
#include <Eigen/Core>

// Graphics
#include <raylib.h>

// Forward declarations
class Variable;
class Simulator;

// Eigen convenience aliases
using Eigen::seqN;
using Eigen::seq;
using Eigen::last;

// Global configuration
extern Globals globals;

//==========================================================================
// FACTOR TYPE ENUMERATION
//==========================================================================
enum FactorType {
    DEFAULT_FACTOR,
    DYNAMICS_FACTOR,
    INTERROBOT_FACTOR,
    OBSTACLE_FACTOR,
    PAYLOAD_TWIST_FACTOR,
    FORCE_ALLOCATION_FACTOR
};

/*****************************************************************************************/
// BASE FACTOR CLASS - Used in Gaussian Belief Propagation (GBP)
/*****************************************************************************************/
class Factor {
public:
    //==========================================================================
    // CORE PROPERTIES
    //==========================================================================
    Simulator* sim_;                            // Pointer to simulator
    int f_id_;                                  // Factor unique identifier
    int r_id_;                                  // Robot ID this factor belongs to
    Key key_;                                   // Factor key = {r_id_, f_id_}
    int other_rid_;                             // Connected robot ID (for inter-robot factors)
    int n_dofs_;                                // DOF of connected variables
    FactorType factor_type_ = DEFAULT_FACTOR;   // Factor type classification

    //==========================================================================
    // MATHEMATICAL COMPONENTS
    //==========================================================================
    Eigen::VectorXd z_;                         // Measurement vector
    Eigen::MatrixXd h_, J_;                     // Measurement function and Jacobian
    Eigen::VectorXd X_;                         // Linearization point
    Eigen::MatrixXd meas_model_lambda_;         // Measurement model precision

    //==========================================================================
    // COMPUTATIONAL STATE
    //==========================================================================
    Mailbox inbox_, outbox_, last_outbox_;      // Message passing mailboxes
    float delta_jac = 1e-8;                     // Delta for numerical Jacobian
    bool initialised_ = false;                  // Jacobian computed flag
    bool linear_ = false;                       // Linear factor flag (avoid Jacobian recomputation)
    bool skip_flag = false;                     // Skip factor update flag

    //==========================================================================
    // CONNECTED VARIABLES
    //==========================================================================
    std::vector<std::shared_ptr<Variable>> variables_;  // Connected variables (order matters)

    //==========================================================================
    // CONSTRUCTOR & DESTRUCTOR
    //==========================================================================
    Factor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
           float sigma, const Eigen::VectorXd& measurement, 
           int n_dofs = 4);
    virtual ~Factor();

    //==========================================================================
    // VIRTUAL INTERFACE (Must be implemented by derived classes)
    //==========================================================================
    virtual Eigen::MatrixXd h_func_(const Eigen::VectorXd& X) = 0;  // Measurement function
    virtual Eigen::MatrixXd J_func_(const Eigen::VectorXd& X);      // Jacobian function
    virtual Eigen::VectorXd residual() { return z_ - h_; }          // Residual computation
    virtual bool skip_factor() {                                    // Skip condition
        skip_flag = false;
        return skip_flag;
    }

    //==========================================================================
    // CORE ALGORITHMS
    //==========================================================================
    bool update_factor();                       // Main factor update algorithm
    Message marginalise_factor_dist(const Eigen::VectorXd& eta, const Eigen::MatrixXd& Lam, 
                                   int var_idx, int marg_idx);  // Marginalization

    //==========================================================================
    // UTILITY METHODS
    //==========================================================================
    Eigen::MatrixXd jacobianFirstOrder(const Eigen::VectorXd& X0);  // Numerical Jacobian
    void draw();                                // Visualization

};

/*****************************************************************************************/
// PAYLOAD COLLABORATION FACTORS
/*****************************************************************************************/

//==========================================================================
// FORCE ALLOCATION FACTOR
// Distributes contact forces among robots to achieve desired payload wrench
//==========================================================================
class ForceAllocationFactor : public Factor {
private:
    // Force mapping and geometry
    Eigen::MatrixXd G_matrix_;                  // Contact mapping matrix [3 x n_contacts]
    Eigen::Vector3d desired_wrench_;            // Desired wrench [fx, fy, τ]
    int n_contact_points_;                      // Number of contact points
    std::vector<Eigen::Vector2d> contact_points_;   // Contact positions (relative to payload center)
    std::vector<Eigen::Vector2d> contact_normals_;  // Contact normal vectors

public:
    ForceAllocationFactor(int f_id, int r_id, 
                         std::vector<std::shared_ptr<Variable>> variables,
                         float sigma, const Eigen::VectorXd& measurement,
                         const std::vector<Eigen::Vector2d>& contact_points,
                         const std::vector<Eigen::Vector2d>& contact_normals);
    
    // Geometry and control updates
    void updateGeometry(const std::vector<Eigen::Vector2d>& contact_points,
                       const std::vector<Eigen::Vector2d>& contact_normals);
    void updateDesiredWrench(const Eigen::Vector3d& desired_wrench);
    
    // Factor interface implementation
    Eigen::MatrixXd h_func_(const Eigen::VectorXd& X) override;
    Eigen::MatrixXd J_func_(const Eigen::VectorXd& X) override;
    
private:
    void computeGMatrix();                      // Compute contact mapping matrix
};

//==========================================================================
// PAYLOAD TWIST FACTOR
// Enforces kinematic consistency between robot motion and payload twist
//==========================================================================
class PayloadTwistFactor : public Factor {
private:
    // Geometric parameters (constant after construction)
    Eigen::Vector2d r_;                         // Contact point relative to payload COM
    Eigen::Vector2d contact_normal_;            // Contact normal vector
    
    // Precomputed geometry (for efficiency)
    Eigen::Vector2d r_perp_;                    // Perpendicular to r vector
    Eigen::Vector2d tangent_;                   // Tangent vector

public:
    PayloadTwistFactor(int f_id, int r_id, 
                       std::vector<std::shared_ptr<Variable>> variables,
                       float sigma, const Eigen::VectorXd& measurement,
                       Eigen::Vector2d r_vector,        // Moment arm vector
                       Eigen::Vector2d normal_vector);  // Contact normal vector
    
    // Geometry updates
    void updateGeometry(const Eigen::Vector2d& r_vector, 
                       const Eigen::Vector2d& normal_vector);
    
    // Factor interface implementation
    Eigen::MatrixXd h_func_(const Eigen::VectorXd& X) override;
    Eigen::MatrixXd J_func_(const Eigen::VectorXd& X) override;
    
private:
    void precomputeGeometry();                  // Precompute geometric quantities
    void precomputeJacobian();                  // Precompute constant Jacobian
};

/*****************************************************************************************/
// ROBOT DYNAMICS & INTERACTION FACTORS
/*****************************************************************************************/

//==========================================================================
// DYNAMICS FACTOR
// Enforces constant-velocity motion model between consecutive robot states
//==========================================================================
class DynamicsFactor : public Factor {
public:
    DynamicsFactor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
                   float sigma, const Eigen::VectorXd& measurement, float dt);

    // Constant velocity model implementation
    Eigen::MatrixXd h_func_(const Eigen::VectorXd& X) override;
    Eigen::MatrixXd J_func_(const Eigen::VectorXd& X) override;
};

//==========================================================================
// INTER-ROBOT FACTOR
// Prevents collisions between robots by penalizing close proximity
//==========================================================================
class InterrobotFactor : public Factor {
private:
    double safety_distance_;                    // Minimum safe distance between robots

public:
    InterrobotFactor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
                     float sigma, const Eigen::VectorXd& measurement, 
                     float robot_radius);

    // Collision avoidance implementation
    Eigen::MatrixXd h_func_(const Eigen::VectorXd& X) override;
    Eigen::MatrixXd J_func_(const Eigen::VectorXd& X) override;
    bool skip_factor() override;                // Skip when robots are far apart
};

//==========================================================================
// OBSTACLE FACTOR
// Prevents robots from entering obstacle regions in the environment
//==========================================================================
class ObstacleFactor : public Factor {
private:
    Image* p_obstacleImage_;                    // Obstacle map image

public:
    ObstacleFactor(Simulator* sim, int f_id, int r_id, 
                   std::vector<std::shared_ptr<Variable>> variables,
                   float sigma, const Eigen::VectorXd& measurement, 
                   Image* p_obstacleImage);

    // Obstacle avoidance implementation
    Eigen::MatrixXd h_func_(const Eigen::VectorXd& X) override;
};

/*****************************************************************************************/
// LEGACY/COMMENTED FACTORS
// These factors are kept commented for reference but not currently used
/*****************************************************************************************/

/*
//==========================================================================
// CONTACT FACTOR (Legacy - commented out)
// Direct contact enforcement between robot and payload
//==========================================================================
class ContactFactor : public Factor {
public:
    ContactFactor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
                  float sigma, const Eigen::VectorXd& measurement,
                  std::shared_ptr<Payload> payload,
                  Eigen::Vector2d target_contact_point,
                  Simulator* sim);
    
    Eigen::MatrixXd h_func_(const Eigen::VectorXd& X) override;
    bool skip_factor() override;
    void draw();
    
    int getPayloadId() const { return payload_->payload_id_; }

private:
    std::shared_ptr<Payload> payload_;
    Eigen::Vector2d target_contact_point_;
    Simulator* sim_;
    
    double computeContactError(const Eigen::Vector2d& robot_pos);
    Eigen::Vector2d payloadToWorld(const Eigen::Vector2d& local_point);
};

//==========================================================================
// PAYLOAD VELOCITY FACTOR (Legacy - commented out)
// Direct velocity control for payload manipulation
//==========================================================================
class PayloadVelocityFactor : public Factor {
public:
    PayloadVelocityFactor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
                          float sigma, const Eigen::VectorXd& measurement,
                          std::shared_ptr<Payload> payload,
                          Eigen::Vector2d contact_normal);
    
    Eigen::MatrixXd h_func_(const Eigen::VectorXd& X) override;
    bool skip_factor() override;
    
    int getPayloadId() const { return payload_->payload_id_; }

private:
    std::shared_ptr<Payload> payload_;
    Eigen::Vector2d contact_normal_;
    
    std::pair<double, double> computeDesiredPayloadMotion();
    std::pair<double, double> computeRobotContribution(const Eigen::Vector2d& robot_pos, 
                                                       const Eigen::Vector2d& robot_velocity);
};
*/