#include "RobotGTSAM.h"
#include "DynamicsFactor.h"
#include <Globals.h>
#include <Simulator.h>
#include <Payload.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <iostream>
#include <cmath>

extern Globals globals;

RobotGTSAM::RobotGTSAM(const gtsam::Vector4& start_state, 
                       const gtsam::Vector4& target_state,
                       Simulator* sim,
                       Color color,
                       float robot_radius,
                       b2World* physicsWorld)
    : sim_(sim), color_(color), robot_radius_(robot_radius), 
      physicsWorld_(physicsWorld), physicsBody_(nullptr), 
      usePhysics_(physicsWorld != nullptr), payload_joint_(nullptr) {
    
    // Calculate parameters from globals (matching GBP logic)
    dt_ = globals.T0;
    std::vector<int> timesteps = getVariableTimesteps(globals.T_HORIZON / globals.T0, globals.LOOKAHEAD_MULTIPLE);
    num_variables_ = timesteps.size();
    
    // Initialize visualization
    initializeVisualization(start_state, target_state);
    
    // Create physics body if physics is enabled
    if (usePhysics_) {
        createPhysicsBody();
    }
    
    createVariables(start_state, target_state);
    createFactors();
}

RobotGTSAM::~RobotGTSAM() {}

std::vector<int> RobotGTSAM::getVariableTimesteps(int lookahead_horizon, int lookahead_multiple) {
    // Replicate GBP's getVariableTimesteps function
    std::vector<int> var_list;
    int N = 1 + int(0.5*(-1 + sqrt(1 + 8*(float)lookahead_horizon/(float)lookahead_multiple)));

    for (int i = 0; i < lookahead_multiple*(N+1); i++) {
        int section = int(i/lookahead_multiple);
        int f = (i - section*lookahead_multiple + lookahead_multiple/2.*section)*(section+1);
        if (f >= lookahead_horizon) {
            var_list.push_back(lookahead_horizon);
            break;
        }
        var_list.push_back(f);
    }

    return var_list;
}

void RobotGTSAM::createVariables(const gtsam::Vector4& start_state, const gtsam::Vector4& target_state) {
    // Get timesteps using GBP logic with globals configuration
    std::vector<int> timesteps = getVariableTimesteps(globals.T_HORIZON / globals.T0, globals.LOOKAHEAD_MULTIPLE);
    
    // Create variables with linear interpolation between start and target
    for (int i = 0; i < num_variables_; i++) {
        gtsam::Symbol key('x', i);
        
        // Linear interpolation: start + t * (target - start)
        float t = (float)timesteps[i] / (float)timesteps.back();
        gtsam::Vector4 interpolated_state = start_state + t * (target_state - start_state);
        
        initial_estimate_.insert(key, interpolated_state);
    }
}

void RobotGTSAM::createFactors() {
    // Use global configuration for noise models (matching GBP)
    gtsam::SharedNoiseModel dynamics_noise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4::Constant(globals.SIGMA_FACTOR_DYNAMICS));
    
    gtsam::SharedNoiseModel prior_noise_strong =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4::Constant(globals.SIGMA_POSE_FIXED));
    
    // Add strong prior on first variable (start state)
    graph_.add(gtsam::PriorFactor<gtsam::Vector4>(
        gtsam::Symbol('x', 0), 
        initial_estimate_.at<gtsam::Vector4>(gtsam::Symbol('x', 0)), 
        prior_noise_strong));
    
    // Add strong prior on last variable (target state)
    graph_.add(gtsam::PriorFactor<gtsam::Vector4>(
        gtsam::Symbol('x', num_variables_ - 1), 
        initial_estimate_.at<gtsam::Vector4>(gtsam::Symbol('x', num_variables_ - 1)), 
        prior_noise_strong));
    
    // Add dynamics factors between consecutive variables
    for (int i = 0; i < num_variables_ - 1; i++) {
        graph_.add(std::make_shared<DynamicsFactor>(
            gtsam::Symbol('x', i), 
            gtsam::Symbol('x', i + 1), 
            dt_, 
            dynamics_noise));
    }
}

void RobotGTSAM::optimize() {
    gtsam::LevenbergMarquardtParams params;
    params.setVerbosity("ERROR");
    
    gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_estimate_, params);
    optimization_result_ = optimizer.optimize();

    std::cout << "=== Optimization Result ===" << std::endl;
    std::cout << "Number of variables: " << num_variables_ << std::endl;
    std::cout << "Number of factors: " << graph_.size() << std::endl;
    optimization_result_.print("State");
    
    // Update visualization with optimized result
    updateVisualization();
}

/********************************************************************************************/
// Visualization Methods (matching Robot.cpp functionality)
/********************************************************************************************/

void RobotGTSAM::initializeVisualization(const gtsam::Vector4& start_state, const gtsam::Vector4& target_state) {
    // Initialize visualization data
    current_position_ = Eigen::Vector2d(start_state(0), start_state(1));
    height_3D_ = robot_radius_;
    interrobot_comms_active_ = true;
    
    // Add start and target as waypoints
    Eigen::VectorXd start_waypoint(4);
    start_waypoint << start_state(0), start_state(1), start_state(2), start_state(3);
    waypoints_.push_back(start_waypoint);
    
    Eigen::VectorXd target_waypoint(4);
    target_waypoint << target_state(0), target_state(1), target_state(2), target_state(3);
    waypoints_.push_back(target_waypoint);
    
    // Initialize trajectory with starting position
    trajectory_.push_back(current_position_);
}

void RobotGTSAM::updateVisualization() {
    if (!optimization_result_.empty()) {
        // Extract current position from first optimized variable
        auto current_state = optimization_result_.at<gtsam::Vector4>(gtsam::Symbol('x', 0));
        current_position_ = Eigen::Vector2d(current_state(0), current_state(1));
        
        // Add to trajectory history
        addTrajectoryPoint(current_position_);
    }
}

void RobotGTSAM::addTrajectoryPoint(const Eigen::Vector2d& point) {
    trajectory_.push_back(point);
    
    // Limit trajectory size for performance (like Robot.cpp)
    if (trajectory_.size() > 1000) {
        trajectory_.erase(trajectory_.begin());
    }
}

void RobotGTSAM::draw() {
    if (!sim_ || !sim_->graphics) return;
    
    // Determine robot color (gray if comms inactive, like Robot.cpp)
    Color col = interrobot_comms_active_ ? color_ : GRAY;
    
    // Draw robot body using 3D model
    DrawModel(sim_->graphics->robotModel_, 
              Vector3{(float)current_position_.x(), height_3D_, (float)current_position_.y()}, 
              robot_radius_, col);
    
    // Draw planned path (spheres for each optimized variable)
    if (globals.DRAW_PATH && !optimization_result_.empty()) {
        for (int i = 0; i < num_variables_; i++) {
            auto state = optimization_result_.at<gtsam::Vector4>(gtsam::Symbol('x', i));
            DrawSphere(Vector3{(float)state(0), height_3D_, (float)state(1)}, 
                       0.5f * robot_radius_, ColorAlpha(col, 0.5f));
        }
    }
    
    // Draw trajectory trail
    if (globals.DRAW_PATH && trajectory_.size() > 1) {
        Color trail_color = ColorAlpha(col, 0.8f);
        for (size_t i = 1; i < trajectory_.size(); i++) {
            Vector3 start = {(float)trajectory_[i-1].x(), 0.1f, (float)trajectory_[i-1].y()};
            Vector3 end = {(float)trajectory_[i].x(), 0.1f, (float)trajectory_[i].y()};
            DrawCylinderEx(start, end, 0.05f, 0.05f, 4, trail_color);
        }
    }
    
    // Draw waypoints
    if (globals.DRAW_WAYPOINTS) {
        for (const auto& waypoint : waypoints_) {
            DrawCubeV(Vector3{(float)waypoint(0), height_3D_, (float)waypoint(1)}, 
                      Vector3{robot_radius_, robot_radius_, robot_radius_}, col);
        }
    }
}

/*******************************************************************************/
// Physics Integration Methods
/*******************************************************************************/

void RobotGTSAM::createPhysicsBody(){
    if (!usePhysics_ || !physicsWorld_) return;
    
    // Create a physics body for the robot
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(current_position_(0), current_position_(1));
    physicsBody_ = physicsWorld_->CreateBody(&bodyDef);

    // Create a circle collision shape for the robot
    b2CircleShape circleShape;
    circleShape.m_radius = robot_radius_;

    // Create a fixture definition for the circle shape
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &circleShape;
    fixtureDef.density = 0.0f;
    fixtureDef.friction = 0.0f;
    fixtureDef.restitution = 0.0f;

    physicsBody_->CreateFixture(&fixtureDef);
}

void RobotGTSAM::syncLogicalToPhysics(){
    if (!usePhysics_ || !physicsBody_) return;
    
    // Get current optimized state [x, y, xdot, ydot]
    gtsam::Vector4 current_state = getCurrentOptimizedState();
    
    // Extract velocity directly from indices 2 and 3 (same as Robot.cpp)
    b2Vec2 desiredVel(current_state(2), current_state(3));
    
    physicsBody_->SetLinearVelocity(desiredVel);
    std::cout << "syncLogicalToPhysics is running..." << std::endl;
}

void RobotGTSAM::syncPhysicsToLogical(){
    if (!usePhysics_ || !physicsBody_) return;

    b2Vec2 phyPos = physicsBody_->GetPosition();
    current_position_[0] = phyPos.x;
    current_position_[1] = phyPos.y;
}

void RobotGTSAM::attachToPayload(std::shared_ptr<Payload> payload, Eigen::Vector2d attach_point) {
    if (!usePhysics_ || !physicsBody_ || !payload) return;
    
    // Get payload's physics body
    b2Body* payload_body = payload->physicsBody_;
    if (!payload_body) return;
    
    // Create weld joint between robot and payload
    b2WeldJointDef jointDef;
    jointDef.bodyA = physicsBody_;
    jointDef.bodyB = payload_body;
    jointDef.localAnchorA.Set(0.0f, 0.0f);  // Robot center
    jointDef.localAnchorB.Set(attach_point.x() - payload->getPosition().x(), 
                             attach_point.y() - payload->getPosition().y());
    jointDef.stiffness = 10000.0f;
    jointDef.damping = 10000.0f;
    
    payload_joint_ = (b2WeldJoint*)physicsWorld_->CreateJoint(&jointDef);
}

void RobotGTSAM::detachFromPayload() {
    if (!usePhysics_ || !payload_joint_) return;
    
    physicsWorld_->DestroyJoint(payload_joint_);
    payload_joint_ = nullptr;
}

gtsam::Vector4 RobotGTSAM::getCurrentOptimizedState() const {
    if (optimization_result_.empty()) {
        // Return current position with zero velocity if no optimization result
        gtsam::Vector4 state;
        state << current_position_(0), current_position_(1), 0.0, 0.0;
        return state;
    }
    
    // Return the first variable (current state) from optimization result
    try {
        return optimization_result_.at<gtsam::Vector4>(gtsam::Symbol('x', 0));
    } catch (const std::exception& e) {
        // Fallback to current position with zero velocity
        gtsam::Vector4 state;
        state << current_position_(0), current_position_(1), 0.0, 0.0;
        return state;
    }
}