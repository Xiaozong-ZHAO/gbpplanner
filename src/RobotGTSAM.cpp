#include "RobotGTSAM.h"
#include "DynamicsFactor.h"

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <iostream>

RobotGTSAM::RobotGTSAM() : dt_(0.1) {
    gtsam::Vector4 priorState = gtsam::Vector4::Zero();
    gtsam::SharedNoiseModel priorNoise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4::Constant(0.1));
    
    graph_.add(gtsam::PriorFactor<gtsam::Vector4>(
        gtsam::Symbol('x', 0), priorState, priorNoise));
    
    initial_estimate_.insert(gtsam::Symbol('x', 0), priorState);
}

RobotGTSAM::~RobotGTSAM() {}

void RobotGTSAM::addState(const gtsam::Vector4& newState) {
    static int idx = 1;

    gtsam::SharedNoiseModel noise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4::Constant(0.1));

    graph_.add(std::make_shared<DynamicsFactor>(
        gtsam::Symbol('x', idx - 1), gtsam::Symbol('x', idx), dt_, noise));
    
    initial_estimate_.insert(gtsam::Symbol('x', idx), newState);
    idx++;
}

void RobotGTSAM::optimize() {
    gtsam::LevenbergMarquardtParams params;
    params.setVerbosity("ERROR");
    
    gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_estimate_, params);
    gtsam::Values result = optimizer.optimize();

    std::cout << "=== Optimization Result ===" << std::endl;
    result.print("State");
}