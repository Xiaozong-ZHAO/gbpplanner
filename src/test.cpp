#include "RobotGTSAM.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <iostream>

int main() {
    std::cout << "Running GTSAM test using RobotGTSAM class..." << std::endl;

    RobotGTSAM robot;

    // 添加两个测试位姿
    gtsam::Pose3 pose1(
        gtsam::Rot3::RzRyRx(0.1, 0.2, 0.1),
        gtsam::Point3(1.0, 0.0, 0.0));

    gtsam::Pose3 pose2(
        gtsam::Rot3::RzRyRx(0.2, 0.1, -0.1),
        gtsam::Point3(2.0, 0.2, 0.1));

    robot.addPose(pose1);
    robot.addPose(pose2);

    robot.optimize();

    std::cout << "GTSAM test completed." << std::endl;
    return 0;
}
