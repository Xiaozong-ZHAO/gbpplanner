#ifndef ROBOT_GTSAM_H
#define ROBOT_GTSAM_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

class RobotGTSAM {
public:
    RobotGTSAM();
    ~RobotGTSAM();

    void addPose(const gtsam::Pose3& pose);
    void optimize();

private:
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
};

#endif // ROBOT_GTSAM_H