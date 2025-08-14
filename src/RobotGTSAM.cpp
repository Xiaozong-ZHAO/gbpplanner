#include "RobotGTSAM.h"

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <iostream>

RobotGTSAM::RobotGTSAM() {
    gtsam::Pose3 priorPose;
    gtsam::SharedNoiseModel priorNoise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Constant(0.1));
    
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
        gtsam::Symbol('x', 0), priorPose, priorNoise));
    
    // 添加初值
    initial_estimate_.insert(gtsam::Symbol('x', 0),
                             gtsam::Pose3::Expmap(gtsam::Vector6::Zero()));
}

RobotGTSAM::~RobotGTSAM() {}

void RobotGTSAM::addPose(const gtsam::Pose3& newPose) {
    static int idx = 1;

    gtsam::SharedNoiseModel noise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Constant(0.1));
    gtsam::Pose3 prevPose = initial_estimate_.at<gtsam::Pose3>(gtsam::Symbol('x', idx - 1));
    gtsam::Pose3 relative = prevPose.between(newPose);

    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        gtsam::Symbol('x', idx - 1), gtsam::Symbol('x', idx), relative, noise));
    
    initial_estimate_.insert(gtsam::Symbol('x', idx), newPose);
    idx++;
}

void RobotGTSAM::optimize() {
    gtsam::LevenbergMarquardtParams params;
    params.setVerbosity("ERROR");
    
    gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_estimate_, params);
    gtsam::Values result = optimizer.optimize();

    std::cout << "=== Optimization Result ===" << std::endl;
    result.print("Pose");
}