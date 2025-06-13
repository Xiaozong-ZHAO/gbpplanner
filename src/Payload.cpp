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
    
    // 获取Box2D的形状信息
    b2Fixture* fixture = physicsBody_->GetFixtureList();
    b2PolygonShape* shape = (b2PolygonShape*)fixture->GetShape();
    b2Transform transform = physicsBody_->GetTransform();
    
    // 获取所有世界坐标的顶点
    std::vector<Eigen::Vector2d> vertices;
    for (int i = 0; i < shape->m_count; i++) {
        b2Vec2 worldVertex = b2Mul(transform, shape->m_vertices[i]);
        vertices.push_back(Eigen::Vector2d(worldVertex.x, worldVertex.y));
    }
    
    // 假设vertices按逆时针顺序：[0]左下, [1]右下, [2]右上, [3]左上
    Eigen::Vector2d center = position_; // payload中心
    
    // 上边：从vertices[3]到vertices[2]
    Eigen::Vector2d topEdgeDir = vertices[2] - vertices[3];
    Eigen::Vector2d topNormal(-topEdgeDir.y(), topEdgeDir.x()); // 垂直向量
    topNormal.normalize();
    
    // 确保法向量指向payload内部
    Eigen::Vector2d topMidpoint = 0.5 * (vertices[2] + vertices[3]);
    if ((center - topMidpoint).dot(topNormal) < 0) {
        topNormal = -topNormal;
    }
    
    // 添加上边的两个接触点和法向量
    points.push_back(vertices[3] + 0.25 * topEdgeDir);
    points.push_back(vertices[3] + 0.75 * topEdgeDir);
    normals.push_back(topNormal);
    normals.push_back(topNormal);
    
    // 左边：从vertices[3]到vertices[0]
    Eigen::Vector2d leftEdgeDir = vertices[0] - vertices[3];
    Eigen::Vector2d leftNormal(-leftEdgeDir.y(), leftEdgeDir.x()); // 垂直向量
    leftNormal.normalize();
    
    // 确保法向量指向payload内部
    Eigen::Vector2d leftMidpoint = 0.5 * (vertices[0] + vertices[3]);
    if ((center - leftMidpoint).dot(leftNormal) < 0) {
        leftNormal = -leftNormal;
    }
    
    // 添加左边的两个接触点和法向量
    points.push_back(vertices[3] + 0.25 * leftEdgeDir);
    points.push_back(vertices[3] + 0.75 * leftEdgeDir);
    normals.push_back(leftNormal);
    normals.push_back(leftNormal);
    
    return {points, normals};
}