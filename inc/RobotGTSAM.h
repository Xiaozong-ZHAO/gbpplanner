#ifndef ROBOT_GTSAM_H
#define ROBOT_GTSAM_H

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <vector>
#include <deque>
#include <raylib.h>
#include <Eigen/Dense>

// Forward declarations
class Simulator;

class RobotGTSAM {
public:
    RobotGTSAM(const gtsam::Vector4& start_state, 
               const gtsam::Vector4& target_state,
               Simulator* sim = nullptr,
               Color color = DARKGREEN,
               float robot_radius = 1.0f);
    ~RobotGTSAM();

    void optimize();
    void draw();
    void updateVisualization();
    void addTrajectoryPoint(const Eigen::Vector2d& point);
    
    // Get variable timesteps (matching GBP logic)
    static std::vector<int> getVariableTimesteps(int lookahead_horizon, int lookahead_multiple);

private:
    // GTSAM optimization
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    gtsam::Values optimization_result_;
    double dt_;
    int num_variables_;
    
    // Visualization data (matching Robot.cpp)
    Simulator* sim_;
    Eigen::Vector2d current_position_;
    std::vector<Eigen::Vector2d> trajectory_;
    std::deque<Eigen::VectorXd> waypoints_;
    Color color_;
    float robot_radius_;
    float height_3D_;
    bool interrobot_comms_active_;
    
    // Helper methods
    void createVariables(const gtsam::Vector4& start_state, const gtsam::Vector4& target_state);
    void createFactors();
    void initializeVisualization(const gtsam::Vector4& start_state, const gtsam::Vector4& target_state);
};

#endif // ROBOT_GTSAM_H