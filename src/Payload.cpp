#include <Payload.h>
#include <Simulator.h>
// #include <Globals.h>

#include <raylib.h>
#include <raymath.h>  // 数学函数
#include <rlgl.h>     // 低级OpenGL包装函数

// extern Globals globals;

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
    target_position_(initial_position), // 默认目标是初始位置
    width_(width),
    height_(height),
    color_(color),
    rotation_(0.0f),
    physicsBody_(nullptr),
    task_completed_(false),
    target_tolerance_(2.0f), // 默认容忍距离
    velocity_(Eigen::Vector2d::Zero()) {
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
    fixtureDef.friction = globals.PAYLOAD_FRICTION;
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

void Payload::setTarget(const Eigen::Vector2d& target) {
    target_position_ = target;
    task_completed_ = false; // 重置任务完成状态
}

void Payload::draw() {
    float x = static_cast<float>(position_.x());
    float y = static_cast<float>(position_.y());
    Vector3 position3D = {x, 0.5f, y};
    Vector3 size = {width_, 1.0f, height_};
    
    // 根据任务状态选择颜色
    Color drawColor = task_completed_ ? GREEN : color_;
    
    rlPushMatrix();
    rlTranslatef(position3D.x, position3D.y, position3D.z);
    rlRotatef(- rotation_ * RAD2DEG, 0, 1, 0);
    DrawCube({0, 0, 0}, size.x, size.y, size.z, drawColor);
    DrawCubeWires({0, 0, 0}, size.x, size.y, size.z, BLACK);
    rlPopMatrix();
    
    // 绘制目标位置
    Vector3 targetPos3D = {static_cast<float>(target_position_.x()), 0.1f, static_cast<float>(target_position_.y())};
    DrawCube(targetPos3D, size.x * 1.1f, 0.2f, size.z * 1.1f, ColorAlpha(RED, 0.3f));
    DrawCubeWires(targetPos3D, size.x * 1.1f, 0.2f, size.z * 1.1f, RED);
    
    // 绘制payload到目标的连线
    if (!task_completed_) {
        Vector3 start = {x, 0.5f, y};
        Vector3 end = {static_cast<float>(target_position_.x()), 0.5f, static_cast<float>(target_position_.y())};
        DrawLine3D(start, end, ColorAlpha(RED, 0.5f));
    }
}


Eigen::Vector2d Payload::getPosition() const{
    return position_;
}

float Payload::getMass() const {
    if (physicsBody_) {
        return physicsBody_->GetMass();
    }
    return 0.0f;
}

double Payload::getMomentOfInertia() const {
    if (physicsBody_) {
        return physicsBody_->GetInertia();
    }
    return 0.0;
}

double Payload::getAngularVelocity() const {
    if (physicsBody_) {
        return physicsBody_->GetAngularVelocity();
    }
    return 0.0;
}

Eigen::Quaterniond Payload::getRotation() const{
    return current_orientation_;
}

Eigen::Vector2d Payload::getVelocity() const {
    if(physicsBody_) {
        b2Vec2 vel = physicsBody_->GetLinearVelocity();
        return Eigen::Vector2d(vel.x, vel.y);
    }
    return Eigen::Vector2d::Zero();
}

std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> Payload::getContactPointsAndNormals() const {
    std::vector<Eigen::Vector2d> points;
    std::vector<Eigen::Vector2d> normals;
    
    if (!physicsBody_) return {points, normals};
    
    // 直接使用payload的几何信息，不依赖Box2D的顶点顺序
    Eigen::Vector2d center = position_;
    double rotation = getRotationFromQuaternion(current_orientation_);
    
    // 旋转矩阵
    Eigen::Matrix2d rot;
    rot << cos(rotation), -sin(rotation),
           sin(rotation),  cos(rotation);
    
    // 在局部坐标系中定义接触点和法向量
    // 上边的两个点
    Eigen::Vector2d local_top1(-width_/4, -height_/2);
    Eigen::Vector2d local_top2(width_/4, -height_/2);
    Eigen::Vector2d local_top_normal(0, 1); // 向payload内部（向下）
    
    // 左边的两个点
    Eigen::Vector2d local_left1(-width_/2, height_/4);
    Eigen::Vector2d local_left2(-width_/2, -height_/4);
    Eigen::Vector2d local_left_normal(1, 0); // 向payload内部（向右）
    
    // 转换到世界坐标系
    points.push_back(center + rot * local_top1);
    points.push_back(center + rot * local_top2);
    points.push_back(center + rot * local_left1);
    points.push_back(center + rot * local_left2);
    
    normals.push_back(rot * local_top_normal);
    normals.push_back(rot * local_top_normal);
    normals.push_back(rot * local_left_normal);
    normals.push_back(rot * local_left_normal);
    
    return {points, normals};
}

// 辅助函数：从四元数提取2D旋转角度
double Payload::getRotationFromQuaternion(const Eigen::Quaterniond& q) const {
    return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 
                 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}