#include "ObstacleFactor.h"
#include "Globals.h"
#include "Payload.h"
#include <cmath>

extern Globals globals;

ObstacleFactor::ObstacleFactor(gtsam::Key key, int obstacle_id, Simulator* sim,
                               const gtsam::SharedNoiseModel& model)
    : gtsam::NoiseModelFactor1<gtsam::Vector4>(model, key), 
      obstacle_id_(obstacle_id), sim_(sim) {
    
    // Cache obstacle parameters (matching GBP ObsFactor constructor)
    obs_pos_ = sim_->obstacles_[obstacle_id_].position;
    obstacle_radius_ = sim_->obstacles_[obstacle_id_].radius;
    padding_ = globals.OBSTACLE_PADDING;
    
    // Calculate payload radius (same as GBP)
    payload_radius_ = sqrt(pow(globals.PAYLOAD_HEIGHT, 2) + pow(globals.PAYLOAD_WIDTH, 2)) / 2.0;
    
    // Calculate distance thresholds (same as GBP)
    D0_ = obstacle_radius_ + payload_radius_;           // Minimum safe distance
    D1_ = obstacle_radius_ + payload_radius_ + padding_; // Distance where penalty starts
}

std::shared_ptr<ObstacleFactor> ObstacleFactor::create(gtsam::Key key, int obstacle_id,
                                                      Simulator* sim, const gtsam::SharedNoiseModel& model) {
    return std::make_shared<ObstacleFactor>(key, obstacle_id, sim, model);
}

gtsam::Vector ObstacleFactor::evaluateError(const gtsam::Vector4& state,
                                           gtsam::OptionalMatrixType H) const {
    
    // Get current payload position (same as GBP)
    auto payload_pos_2d = sim_->payloads_.begin()->second->getPosition();
    Eigen::Vector2d payload_pos(payload_pos_2d.x(), payload_pos_2d.y());
    double d = (payload_pos - obs_pos_).norm();
    if (H) {
        *H = gtsam::Matrix::Zero(1, 4); // Jacobian will be 1x4
        if (d <= D1_) {
            // Compute Jacobian
            Eigen::Vector2d n = (payload_pos - obs_pos_) / d; // Unit vector from obstacle to payload
            (*H)(0, 0) = -n.x() / padding_;  // ∂h/∂x
            (*H)(0, 1) = -n.y() / padding_;  // ∂h/∂y
        }
    }
    // Robot position from state
    Eigen::Vector2d robot_pos = state.head<2>();
    
    // Compute constraint value (same logic as GBP ObsFactor::h_func_)
    double constraint = computeConstraint(robot_pos, payload_pos);
    
    // Compute Jacobian if requested
    // if (H) {
    gtsam::Vector2 jac_pos = computeJacobian(robot_pos, payload_pos);
        
    // H is 1x4 matrix: [∂h/∂x, ∂h/∂y, ∂h/∂vx, ∂h/∂vy]
    // Only position affects obstacle constraint, velocity doesn't
    *H = gtsam::Matrix::Zero(1, 4);
    (*H)(0, 0) = jac_pos(0);  // ∂h/∂x
    (*H)(0, 1) = jac_pos(1);  // ∂h/∂y
        // (*H)(0, 2) = 0;        // ∂h/∂vx = 0
        // (*H)(0, 3) = 0;        // ∂h/∂vy = 0
    // }
    
    // Return constraint as 1D vector (GTSAM expects Vector, GBP returns scalar)
    gtsam::Vector1 error;
    error << constraint;
    return error;
}

double ObstacleFactor::computeConstraint(const Eigen::Vector2d& robot_pos, 
                                        const Eigen::Vector2d& payload_pos) const {
    
    // Calculate distance from payload to obstacle (same as GBP)
    double d = (payload_pos - obs_pos_).norm();
    
    // Apply same constraint logic as GBP ObsFactor::h_func_
    if (d <= D1_ && d >= D0_) {
        // Linear falloff between D1 and D0
        return 1.0 * (1.0 - (d - D0_) / padding_);
    }
    else if (d < D0_) {
        // Obstacle is too close, large penalty
        return 1.0;
    }
    else {
        // Obstacle is far enough, no penalty
        return 0.0;
    }
}

gtsam::Vector2 ObstacleFactor::computeJacobian(const Eigen::Vector2d& robot_pos,
                                              const Eigen::Vector2d& payload_pos) const {
    
    // Calculate distance from payload to obstacle
    double d = (payload_pos - obs_pos_).norm();
    
    gtsam::Vector2 jacobian = gtsam::Vector2::Zero();
    
    // Only compute Jacobian if we're in the penalty region (same as GBP)
    if (d <= D1_) {
        // Unit vector from obstacle to payload
        Eigen::Vector2d n = (payload_pos - obs_pos_) / d;
        
        // Jacobian with respect to robot position (same as GBP ObsFactor::J_func_)
        // Note: In GBP, this affects payload position through robot-payload coupling
        jacobian(0) = -n.x() / padding_;  // ∂h/∂x
        jacobian(1) = -n.y() / padding_;  // ∂h/∂y
    }
    
    return jacobian;
}