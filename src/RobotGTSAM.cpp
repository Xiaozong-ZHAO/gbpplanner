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

RobotGTSAM::RobotGTSAM(Simulator* sim, 
                       int robot_id,
                       std::deque<Eigen::VectorXd> waypoints,
                       float robot_radius,
                       Color color,
                       b2World* physicsWorld,
                       Payload* payload,
                       const Eigen::Vector2d& contact_point)
    : sim_(sim), robot_id_(robot_id), waypoints_(waypoints), 
      color_(color), robot_radius_(robot_radius), 
      physicsWorld_(physicsWorld), physicsBody_(nullptr), 
      usePhysics_(physicsWorld != nullptr), payload_joint_(nullptr),
      payload_(payload), contact_point_local_(contact_point) {
    
    // Calculate parameters from globals (matching GBP logic)
    timesteps_ = getVariableTimesteps(globals.T_HORIZON / globals.T0, globals.LOOKAHEAD_MULTIPLE);
    num_variables_ = timesteps_.size();
    
    // Extract start and target from waypoints (matching Robot.cpp:30-32)
    start_waypoint_ = waypoints_[0];  // First waypoint = start (store for createVariables)
    waypoints_.pop_front();           // Remove start waypoint
    // Target waypoint remains in waypoints_.front() for updateHorizon()
    
    // Initialize visualization
    initializeVisualization();
    
    // Create physics body if physics is enabled
    if (usePhysics_) {
        createPhysicsBody();
    }
    
    createVariables();
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

void RobotGTSAM::createVariables() {
    // Use stored timesteps_ instead of recalculating
    
    // Use start_waypoint_ and waypoints_.front() as target (after pop_front in constructor)
    Eigen::VectorXd target_waypoint = waypoints_.empty() ? Eigen::VectorXd::Zero(4) : waypoints_.front();  // Target state
    
    gtsam::Vector4 start_state;
    start_state << start_waypoint_(0), start_waypoint_(1), start_waypoint_(2), start_waypoint_(3);
    gtsam::Vector4 target_state;
    target_state << target_waypoint(0), target_waypoint(1), target_waypoint(2), target_waypoint(3);
    
    // Create variables with payload-coupled trajectory planning (matching Robot.cpp:65-120)
    for (int i = 0; i < num_variables_; i++) {
        gtsam::Symbol key('x', i);
        gtsam::Vector4 interpolated_state;
        
        if (payload_ && globals.FORMATION == "Payload" && globals.T_HORIZON > 0) {
            // Get payload contact points (matching Robot.cpp logic)
            auto [contact_points, contact_normals] = payload_->getContactPointsAndNormals();
            
            if (contact_points.empty()) {
                // Fallback to linear interpolation if no contact points
                float t = (float)timesteps_[i] / (float)timesteps_.back();
                interpolated_state = start_state + t * (target_state - start_state);
            } else {
                // Rigid body interpolation for payload formation
                float t = (float)(timesteps_[i] * globals.T0 / globals.T_HORIZON);
                
                // Get payload motion parameters
                Eigen::Vector2d P_start = payload_->getPosition();
                Eigen::Vector2d P_target = payload_->getTarget();
                Eigen::Quaterniond q_start = payload_->current_orientation_;
                Eigen::Quaterniond q_target = payload_->getTargetRotation();
                
                // Convert quaternions to rotation angles  
                double θ_start = atan2(2.0 * (q_start.w() * q_start.z() + q_start.x() * q_start.y()),
                                      1.0 - 2.0 * (q_start.y() * q_start.y() + q_start.z() * q_start.z()));
                double θ_target = atan2(2.0 * (q_target.w() * q_target.z() + q_target.x() * q_target.y()),
                                       1.0 - 2.0 * (q_target.y() * q_target.y() + q_target.z() * q_target.z()));
                
                // Interpolate payload state
                Eigen::Vector2d P_t = P_start + (P_target - P_start) * t;
                double θ_t = θ_start + (θ_target - θ_start) * t;
                
                // Use contact point from constructor parameter (r_i in payload frame)
                Eigen::Vector2d r_i = contact_point_local_;
                
                // Rotation matrix at time t
                Eigen::Matrix2d R_t;
                R_t << cos(θ_t), -sin(θ_t),
                       sin(θ_t),  cos(θ_t);
                
                // Robot position maintaining rigid body constraint
                Eigen::Vector2d robot_pos_t = P_t + R_t * r_i;
                
                // Robot velocity from rigid body motion  
                double T_total = globals.T_HORIZON;
                Eigen::Vector2d v_payload = (P_target - P_start) / T_total;
                double ω_payload = (θ_target - θ_start) / T_total;
                
                Eigen::Vector2d r_i_rotated = R_t * r_i;
                Eigen::Vector2d robot_vel_t = v_payload + ω_payload * Eigen::Vector2d(-r_i_rotated.y(), r_i_rotated.x());
                
                // Set complete 4-DOF state
                interpolated_state << robot_pos_t.x(), robot_pos_t.y(), robot_vel_t.x(), robot_vel_t.y();
            }
        } else {
            // Original linear interpolation for non-payload formations
            float t = (float)timesteps_[i] / (float)timesteps_.back();
            interpolated_state = start_state + t * (target_state - start_state);
        }
        
        initial_estimate_.insert(key, interpolated_state);
    }
}

void RobotGTSAM::createFactors() {
    // Use global configuration for noise models (matching GBP)
    gtsam::SharedNoiseModel dynamics_noise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4::Constant(globals.SIGMA_FACTOR_DYNAMICS));
    
    // Create updateable prior noise models (matching GBP variable priors)
    current_prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4::Constant(globals.SIGMA_POSE_FIXED));
    horizon_prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4::Constant(globals.SIGMA_POSE_FIXED));
    
    // Add updateable prior on first variable (current state)
    current_prior_factor_key_ = graph_.size();
    graph_.add(gtsam::PriorFactor<gtsam::Vector4>(
        gtsam::Symbol('x', 0), 
        initial_estimate_.at<gtsam::Vector4>(gtsam::Symbol('x', 0)), 
        current_prior_noise_));
    
    // Add updateable prior on last variable (horizon state)
    horizon_prior_factor_key_ = graph_.size();
    graph_.add(gtsam::PriorFactor<gtsam::Vector4>(
        gtsam::Symbol('x', num_variables_ - 1), 
        initial_estimate_.at<gtsam::Vector4>(gtsam::Symbol('x', num_variables_ - 1)), 
        horizon_prior_noise_));
    
    // Add dynamics factors between consecutive variables with variable dt (matching Robot.cpp)
    for (int i = 0; i < num_variables_ - 1; i++) {
        double delta_t = globals.T0 * (timesteps_[i + 1] - timesteps_[i]);  // Variable time spacing
        graph_.add(std::make_shared<DynamicsFactor>(
            gtsam::Symbol('x', i), 
            gtsam::Symbol('x', i + 1), 
            delta_t, 
            dynamics_noise));
    }
}

/********************************************************************************************/
// Main Interface Methods (matching Robot.cpp)
/********************************************************************************************/

void RobotGTSAM::updateCurrent() {
    // Move plan: move plan current state by plan increment
    gtsam::Vector4 current_state = getCurrentOptimizedState(0);  // x₀ 
    gtsam::Vector4 next_state = getCurrentOptimizedState(1);     // x₁
    gtsam::Vector4 increment = (next_state - current_state) * globals.TIMESTEP / globals.T0;
    
    // In GBP we do this by modifying the prior on the variable
    gtsam::Vector4 new_prior = current_state + increment;
    updateCurrentPrior(new_prior);
    
    // Real pose update
    current_position_ = current_position_ + increment.head<2>().cast<double>();
    
    // Track trajectory for visualization
    addTrajectoryPoint(Eigen::Vector2d(current_position_(0), current_position_(1)));
    
    // Update physics body state to match logical state
    syncLogicalToPhysics();
}

void RobotGTSAM::updateHorizon() {
    // Horizon state moves towards the next waypoint.
    // The Horizon state's velocity is capped at MAX_SPEED
    if (waypoints_.empty()) {
        return;  // No waypoints to pursue
    }
    
    gtsam::Vector4 horizon_state = getCurrentOptimizedState(num_variables_ - 1);
    Eigen::VectorXd waypoint_target = waypoints_.front();
    Eigen::Vector2d dist_horz_to_goal = waypoint_target.head<2>() - horizon_state.head<2>().cast<double>();
    Eigen::Vector2d new_vel = dist_horz_to_goal.normalized() * std::min((double)globals.MAX_SPEED, dist_horz_to_goal.norm());
    Eigen::Vector2d new_pos = horizon_state.head<2>().cast<double>() + new_vel * globals.TIMESTEP;

    gtsam::Vector4 new_horizon;
    new_horizon << new_pos.x(), new_pos.y(), new_vel.x(), new_vel.y();
    updateHorizonPrior(new_horizon);

    // If the horizon has reached the waypoint, pop that waypoint from the waypoints.
    if (dist_horz_to_goal.norm() < robot_radius_) {
        waypoints_.pop_front();
    }
}

/********************************************************************************************/
// Internal Implementation Methods
/********************************************************************************************/

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

void RobotGTSAM::initializeVisualization() {
    // Initialize visualization data using start waypoint
    current_position_ = Eigen::Vector2d(start_waypoint_(0), start_waypoint_(1));
    
    height_3D_ = robot_radius_;
    interrobot_comms_active_ = true;
    
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
    physicsBody_->SetTransform(b2Vec2(current_position_(0), current_position_(1)), 0.0f);

    // Get current optimized state [x, y, xdot, ydot]
    gtsam::Vector4 current_state = getCurrentOptimizedState();
    
    // Extract velocity directly from indices 2 and 3 (same as Robot.cpp)
    b2Vec2 desiredVel(current_state(2), current_state(3));
    
    physicsBody_->SetLinearVelocity(desiredVel);
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
    return getCurrentOptimizedState(0);  // Default to index 0
}

gtsam::Vector4 RobotGTSAM::getCurrentOptimizedState(int var_index) const {
    if (optimization_result_.empty()) {
        // Return current position with zero velocity if no optimization result
        gtsam::Vector4 state;
        state << current_position_(0), current_position_(1), 0.0, 0.0;
        return state;
    }
    
    // Return the specified variable from optimization result
    try {
        if (var_index >= num_variables_) {
            // If requesting beyond available variables, return last variable
            var_index = num_variables_ - 1;
        }
        return optimization_result_.at<gtsam::Vector4>(gtsam::Symbol('x', var_index));
    } catch (const std::exception& e) {
        // Fallback to current position with zero velocity
        gtsam::Vector4 state;
        state << current_position_(0), current_position_(1), 0.0, 0.0;
        return state;
    }
}

void RobotGTSAM::updateVariablePrior(int var_index, const gtsam::Vector4& new_prior) {
    // GTSAM equivalent of Robot.cpp's change_variable_prior()
    // This updates the prior factor on the specified variable
    
    if (var_index >= num_variables_) {
        std::cerr << "Warning: Variable index " << var_index << " exceeds available variables" << std::endl;
        return;
    }
    
    try {
        // For now, we'll update the initial estimate and re-optimize when needed
        // In a more sophisticated implementation, we could remove the old prior factor
        // and add a new one, but this requires more complex factor graph manipulation
        initial_estimate_.update(gtsam::Symbol('x', var_index), new_prior);
        
        // Update optimization result to reflect the new prior
        if (!optimization_result_.empty()) {
            optimization_result_.update(gtsam::Symbol('x', var_index), new_prior);
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error updating variable prior: " << e.what() << std::endl;
    }
}

void RobotGTSAM::updateCurrentPrior(const gtsam::Vector4& new_prior) {
    // Remove old current prior factor and add new one (matching GBP change_variable_prior)
    graph_.remove(current_prior_factor_key_);
    current_prior_factor_key_ = graph_.size();
    graph_.add(gtsam::PriorFactor<gtsam::Vector4>(
        gtsam::Symbol('x', 0), new_prior, current_prior_noise_));
    
    // Also update initial estimate
    initial_estimate_.update(gtsam::Symbol('x', 0), new_prior);
}

void RobotGTSAM::updateHorizonPrior(const gtsam::Vector4& new_horizon) {
    // Remove old horizon prior factor and add new one (matching GBP change_variable_prior)
    graph_.remove(horizon_prior_factor_key_);
    horizon_prior_factor_key_ = graph_.size();
    graph_.add(gtsam::PriorFactor<gtsam::Vector4>(
        gtsam::Symbol('x', num_variables_ - 1), new_horizon, horizon_prior_noise_));
    
    // Also update initial estimate
    initial_estimate_.update(gtsam::Symbol('x', num_variables_ - 1), new_horizon);
}

void RobotGTSAM::shiftTrajectoryHorizon() {
    // Shift planned trajectory forward in time
    // This matches Robot.cpp's approach of maintaining a moving horizon
    
    if (optimization_result_.empty() || num_variables_ <= 1) {
        return;
    }
    
    try {
        // Shift all variables: x[i] ← x[i+1]
        // The last variable will be duplicated (could be improved with re-planning)
        for (int i = 0; i < num_variables_ - 1; i++) {
            gtsam::Vector4 next_state = optimization_result_.at<gtsam::Vector4>(gtsam::Symbol('x', i + 1));
            initial_estimate_.update(gtsam::Symbol('x', i), next_state);
            optimization_result_.update(gtsam::Symbol('x', i), next_state);
        }
        
        // For the last variable, we could extrapolate or keep the target
        // For now, keep the target state
        gtsam::Vector4 last_state = optimization_result_.at<gtsam::Vector4>(gtsam::Symbol('x', num_variables_ - 1));
        initial_estimate_.update(gtsam::Symbol('x', num_variables_ - 1), last_state);
        
    } catch (const std::exception& e) {
        std::cerr << "Error shifting trajectory horizon: " << e.what() << std::endl;
    }
}