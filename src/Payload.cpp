#include <Payload.h>
#include <Simulator.h>
#include <Globals.h>

#include <raylib.h>
#include <raymath.h>  // 数学函数
#include <rlgl.h>     // 低级OpenGL包装函数

extern Globals globals;

Payload::Payload(Simulator* sim,
                 int payload_id,
                 Eigen::Vector2d initial_position,
                 float width,
                 float height,
                 float density,
                 Color color) :
    sim_(sim),
    payload_id_(payload_id),
    position_(initial_position),
    width_(width),
    height_(height),
    color_(color),
    rotation_(0.0f),
    physicsBody_(nullptr){
        physicsWorld_ = sim->getPhysicsWorld();
        if (physicsWorld_){
            createPhysicsBody(density);

        }
    }
Payload::~Payload(){
    if (physicsBody_){
        physicsWorld_->DestroyBody(physicsBody_);
    }
}

void Payload::createPhysicsBody(float density){
    // Initialize a payload physics body
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(position_(0), position_(1));
    bodyDef.angle = rotation_;

    // Create the body in the physics world
    physicsBody_ = physicsWorld_->CreateBody(&bodyDef);

    // Create a rectangle shape for the payload
    b2PolygonShape boxShape;
    boxShape.SetAsBox(width_ / 2.0f, height_ / 2.0f);

    // Create a fixture definition for the rectangle shape
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &boxShape;
    fixtureDef.density = density;
    fixtureDef.friction = 0.05f;
    fixtureDef.restitution = 0.1f;
    
    physicsBody_->CreateFixture(&fixtureDef);
    
    // 设置阻尼
    physicsBody_->SetLinearDamping(0.2f);   // 线性阻尼
    physicsBody_->SetAngularDamping(0.3f);  // 角度阻
}

void Payload::update() {
    if (physicsBody_) {
        b2Vec2 physpos = physicsBody_->GetPosition();
        position_.x() = physpos.x;
        position_.y() = physpos.y;
        rotation_ = physicsBody_->GetAngle();
    }
}

void Payload::draw() {
    // 绘制3D立方体表示payload
    Vector3 position3D = {position_.x(), 0.5f, position_.y()};  // 稍微抬高一点
    Vector3 size = {width_, 1.0f, height_};
    
    // 使用旋转矩阵绘制
    rlPushMatrix();
    rlTranslatef(position3D.x, position3D.y, position3D.z);
    rlRotatef(rotation_ * RAD2DEG, 0, 1, 0);  // 绕Y轴旋转
    DrawCube({0, 0, 0}, size.x, size.y, size.z, color_);
    DrawCubeWires({0, 0, 0}, size.x, size.y, size.z, BLACK);
    rlPopMatrix();
}


Eigen::Vector2d Payload::getPosition() const{
    return position_;
}

float Payload::getRotation() const{
    return rotation_;
}

Eigen::Vector2d Payload::getVelocity() const {
    if(physicsBody_) {
        b2Vec2 vel = physicsBody_->GetLinearVelocity();
        return Eigen::Vector2d(vel.x, vel.y);
    }
    return Eigen::Vector2d::Zero();
}

