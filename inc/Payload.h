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
                float density = 1.0f,
                Color color = GRAY);
        ~Payload();

    // Fundamental properties
    int payload_id_;
    Eigen::Vector2d position_;
    float width_, height_;
    float rotation_;
    Color color_;

    // Box2D properties
    b2Body* physicsBody_;
    b2World* physicsWorld_;

    // update and draw functions
    void update();
    void draw();
    
    // Create the physical entity
    void createPhysicsBody(float density);
    
    Eigen::Vector2d getPosition() const;
    float getRotation() const;
    Eigen::Vector2d getVelocity() const;

    private:
        Simulator* sim_;
};

# endif // PAYLOAD_H