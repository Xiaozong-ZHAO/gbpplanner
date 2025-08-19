#ifndef OBSTACLE_FACTOR_H
#define OBSTACLE_FACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Key.h>
#include <Eigen/Dense>
#include "Simulator.h"

/**
 * ObstacleFactor implements obstacle avoidance constraints for GTSAM optimization.
 * This factor creates a penalty when a robot variable gets too close to a static obstacle.
 * 
 * The constraint formulation matches the GBP ObsFactor:
 * - Uses linear falloff between D0 and D1 distances
 * - D0 = obstacle_radius + payload_radius (minimum safe distance)
 * - D1 = D0 + padding (distance where penalty starts)
 * - Penalty is based on payload position relative to obstacle
 */
class ObstacleFactor : public gtsam::NoiseModelFactor1<gtsam::Vector4> {
public:
    ObstacleFactor(gtsam::Key key, int obstacle_id, Simulator* sim, 
                   const gtsam::SharedNoiseModel& model);

    gtsam::Vector evaluateError(const gtsam::Vector4& state, 
                               gtsam::OptionalMatrixType H = OptionalNone) const override;

    /**
     * Create a shared pointer to this factor
     */
    static std::shared_ptr<ObstacleFactor> create(gtsam::Key key, int obstacle_id, 
                                                 Simulator* sim, const gtsam::SharedNoiseModel& model);

private:
    int obstacle_id_;           // ID of the obstacle
    Simulator* sim_;           // Pointer to simulator
    
    // Cached obstacle parameters (computed in constructor)
    Eigen::Vector2d obs_pos_;  // Obstacle position
    double obstacle_radius_;   // Obstacle radius
    double payload_radius_;    // Payload radius
    double padding_;           // Safety padding
    double D0_;               // Minimum safe distance
    double D1_;               // Distance where penalty starts
    
    /**
     * Compute the constraint value (matches GBP ObsFactor::h_func_)
     */
    double computeConstraint(const Eigen::Vector2d& robot_pos, const Eigen::Vector2d& payload_pos) const;
    
    /**
     * Compute the Jacobian (matches GBP ObsFactor::J_func_)
     */
    gtsam::Vector2 computeJacobian(const Eigen::Vector2d& robot_pos, const Eigen::Vector2d& payload_pos) const;
};

#endif // OBSTACLE_FACTOR_H