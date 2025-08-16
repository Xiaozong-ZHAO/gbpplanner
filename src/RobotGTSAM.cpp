#include "RobotGTSAM.h"
#include "DynamicsFactor.h"
#include <Globals.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <iostream>
#include <cmath>

extern Globals globals;

RobotGTSAM::RobotGTSAM(const gtsam::Vector4& start_state, 
                       const gtsam::Vector4& target_state) {
    
    // Calculate parameters from globals (matching GBP logic)
    dt_ = globals.T0;
    std::vector<int> timesteps = getVariableTimesteps(globals.T_HORIZON / globals.T0, globals.LOOKAHEAD_MULTIPLE);
    num_variables_ = timesteps.size();
    
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
    gtsam::Values result = optimizer.optimize();

    std::cout << "=== Optimization Result ===" << std::endl;
    std::cout << "Number of variables: " << num_variables_ << std::endl;
    std::cout << "Number of factors: " << graph_.size() << std::endl;
    result.print("State");
}