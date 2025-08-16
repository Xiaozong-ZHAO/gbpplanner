#ifndef ROBOT_GTSAM_H
#define ROBOT_GTSAM_H

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <vector>

class RobotGTSAM {
public:
    RobotGTSAM(const gtsam::Vector4& start_state, 
               const gtsam::Vector4& target_state, 
               int num_variables = 24,
               double dt = 0.1);
    ~RobotGTSAM();

    void optimize();
    
    // Get variable timesteps (matching GBP logic)
    static std::vector<int> getVariableTimesteps(int lookahead_horizon, int lookahead_multiple);

private:
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    double dt_;
    int num_variables_;
    
    // Helper methods
    void createVariables(const gtsam::Vector4& start_state, const gtsam::Vector4& target_state);
    void createFactors();
};

#endif // ROBOT_GTSAM_H