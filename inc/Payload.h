#ifndef PAYLOAD_H
#define PAYLOAD_H

#include "box2d/box2d.h"
#include <Eigen/Dense>
#include <memory>
#include <raylib.h>

class Simulator;

class Payload {
    public:
        Payload(Simulator* sim,
                int payload_id,
                Eigen::Vector2d initial_position,
                float width,
                float height,
                float density,
                Color color = GRAY);
        ~Payload();

    // Fundamental properties
    int payload_id_;
    Eigen::Vector2d position_;
    Eigen::Vector2d target_position_;
    Eigen::Quaterniond initial_orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond current_orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond target_orientation_ = Eigen::Quaterniond::Identity();
    double mass_;
    double moment_of_inertia_;
    double current_angular_velocity_;
    float width_, height_;
    float rotation_;
    Color color_;

    // Task related properties
    bool task_completed_;              
    float target_tolerance_;           
    Eigen::Vector2d velocity_;         

    // Box2D properties
    b2Body* physicsBody_;
    b2World* physicsWorld_;

    // update and draw functions
    void update();
    void draw();
    
    // Create the physical entity
    void createPhysicsBody(float density);
    std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> getContactPointsAndNormals() const;
    double getRotationFromQuaternion(const Eigen::Quaterniond& q) const;
    

    void setTarget(const Eigen::Vector2d& target);
    void setTarget(const Eigen::Vector2d& target_position, const Eigen::Quaterniond& target_orientation);
    bool isAtTarget() const;
    Eigen::Vector2d getDistanceToTarget() const;
    float getDistanceToTargetMagnitude() const;
    double getAngularVelocity() const;
    float getMass() const;
    double getMomentOfInertia() const;
    Eigen::Vector2d getTarget();
    Eigen::Vector2d getRequiredPushDirection() const;
    bool shouldStopPushing() const;
    
    Eigen::Vector2d getPosition() const;
    Eigen::Quaterniond getTargetRotation() const;
    Eigen::Vector2d getVelocity() const;
    double getRotationError() const;

    void setTargetFromRelativeRotation(double relative_rotation_rad);  // 新增方法
    void updateTargetOrientation();  // 新增：更新目标朝向的方法

    private:
        Simulator* sim_;
};

# endif // PAYLOAD_H