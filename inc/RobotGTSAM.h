#ifndef ROBOT_GTSAM_H
#define ROBOT_GTSAM_H

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

class RobotGTSAM {
public:
    RobotGTSAM();
    ~RobotGTSAM();

    void addState(const gtsam::Vector4& state);
    void optimize();

private:
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    double dt_;  // Time step for dynamics
};

#endif // ROBOT_GTSAM_H