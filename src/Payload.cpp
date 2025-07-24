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
    target_orientation_(Eigen::Quaterniond::Identity()),
    velocity_(Eigen::Vector2d::Zero()) {
        trajectory_history_.reserve(MAX_TRAJECTORY_POINTS);
        trajectory_history_.push_back(initial_position);
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
    fixtureDef.friction = 0.0f;
    fixtureDef.restitution = 0.0f;
    
    physicsBody_->CreateFixture(&fixtureDef);
    
    // // 设置阻尼
    // physicsBody_->SetLinearDamping(0.0f);   // 线性阻尼
    // physicsBody_->SetAngularDamping(0.0f);  // 角度阻
}

void Payload::setTargetFromRelativeRotation(double relative_rotation_rad) {
    // target_orientation = initial_orientation * relative_rotation
    Eigen::Quaterniond relative_rotation(cos(relative_rotation_rad/2), 0, 0, sin(relative_rotation_rad/2));
    target_orientation_ = initial_orientation_ * relative_rotation;
    
    std::cout << "Target orientation set: initial + " << relative_rotation_rad 
              << " rad = " << getRotationFromQuaternion(target_orientation_) << " rad" << std::endl;
}

void Payload::update() {
    if (physicsBody_) {
        b2Vec2 physpos = physicsBody_->GetPosition();
        position_.x() = physpos.x;
        position_.y() = physpos.y;
        rotation_ = physicsBody_->GetAngle();
        
        // 正确更新current_orientation_
        current_orientation_ = Eigen::Quaterniond(cos(rotation_/2), 0, 0, sin(rotation_/2));
        
        // Record trajectory
        if (trajectory_history_.size() >= MAX_TRAJECTORY_POINTS) {
            trajectory_history_.erase(trajectory_history_.begin());
        }
        trajectory_history_.push_back(position_);
    }
}

void Payload::setTarget(const Eigen::Vector2d& target) {
    target_position_ = target;
    task_completed_ = false; // 重置任务完成状态
}

void Payload::setTarget(const Eigen::Vector2d& target_position, const Eigen::Quaterniond& target_orientation) {
    target_position_ = target_position;
    target_orientation_ = target_orientation;
    task_completed_ = false;
}

Eigen::Vector2d Payload::getTarget(){
    return target_position_;
}

void Payload::draw() {
    float x = static_cast<float>(position_.x());
    float y = static_cast<float>(position_.y());
    
    // Draw trajectory
    if (trajectory_history_.size() > 1) {
        for (size_t i = 1; i < trajectory_history_.size(); ++i) {
            Vector3 start = {static_cast<float>(trajectory_history_[i-1].x()), 0.1f, static_cast<float>(trajectory_history_[i-1].y())};
            Vector3 end = {static_cast<float>(trajectory_history_[i].x()), 0.1f, static_cast<float>(trajectory_history_[i].y())};
            DrawLine3D(start, end, ColorAlpha(BLUE, 0.7f));
        }
    }
    Vector3 position3D = {x, 0.5f, y};
    Vector3 size = {width_, 1.0f, height_};
    float radius = sqrt(width_ * width_ + height_ * height_) / 2.0f;
    
    // 根据任务状态选择颜色
    Color drawColor = task_completed_ ? GREEN : color_;
    
    // 绘制当前payload
    rlPushMatrix();
    rlTranslatef(position3D.x, position3D.y, position3D.z);
    rlRotatef(- rotation_ * RAD2DEG, 0, 1, 0);
    DrawCube({0, 0, 0}, size.x, size.y, size.z, drawColor);
    DrawCubeWires({0, 0, 0}, size.x, size.y, size.z, BLACK);
    rlPopMatrix();
    
    // 绘制目标位置（保持原有逻辑）
    Vector3 targetPos3D = {static_cast<float>(target_position_.x()), 0.1f, static_cast<float>(target_position_.y())};
    
    // 新增：绘制带有目标朝向的目标位置
    rlPushMatrix();
    rlTranslatef(targetPos3D.x, targetPos3D.y, targetPos3D.z);
    
    // 计算目标朝向角度
    double target_angle = getRotationFromQuaternion(target_orientation_);
    rlRotatef(-target_angle * RAD2DEG, 0, 1, 0);  // 应用目标朝向
    
    // 绘制目标位置的立方体（带朝向）
    DrawCube({0, 0, 0}, size.x * 1.1f, 0.2f, size.z * 1.1f, ColorAlpha(RED, 0.3f));
    DrawCubeWires({0, 0, 0}, size.x * 1.1f, 0.2f, size.z * 1.1f, RED);
    rlPopMatrix();

    // draw a circle with the variable radius in 3D
    Vector3 circle_center = {x, 0.1f, y};
    DrawCircle3D(circle_center, radius, {1, 0, 0}, 90.0f, ColorAlpha(GREEN, 0.5f));

    
    // 绘制payload到目标的连线（保持原有逻辑）
    if (!task_completed_) {
        Vector3 start = {x, 0.5f, y};
        Vector3 end = {static_cast<float>(target_position_.x()), 0.5f, static_cast<float>(target_position_.y())};
        DrawLine3D(start, end, ColorAlpha(RED, 0.5f));
    }
    
    // 新增：绘制目标朝向指示箭头
    if (std::abs(getRotationError()) > 0.01) {
        double target_angle = getRotationFromQuaternion(target_orientation_);
        Vector3 target_center = {static_cast<float>(target_position_.x()), 0.3f, static_cast<float>(target_position_.y())};
        
        // 箭头长度为payload宽度的一半
        float arrow_length = std::max(width_, height_) * 0.5f;
        Vector3 arrow_end = {
            target_center.x + arrow_length * cos(target_angle),
            target_center.y,
            target_center.z + arrow_length * sin(target_angle)
        };
        
        // 绘制朝向箭头
        DrawLine3D(target_center, arrow_end, ORANGE);
        DrawSphere(arrow_end, 0.8f, ORANGE);  // 箭头头部
        
    }
    
    // 新增：绘制当前朝向指示箭头（用于对比）
    {
        Vector3 current_center = {x, 0.7f, y};  // 稍微高一点避免重叠
        
        float arrow_length = std::max(width_, height_) * 0.4f;
        Vector3 current_arrow_end = {
            current_center.x + arrow_length * cos(rotation_),
            current_center.y,
            current_center.z + arrow_length * sin(rotation_)
        };
        
        // 绘制当前朝向箭头（绿色）
        DrawLine3D(current_center, current_arrow_end, GREEN);
        DrawSphere(current_arrow_end, 0.6f, GREEN);  // 箭头头部
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

Eigen::Quaterniond Payload::getTargetRotation() const{
    return target_orientation_;
}

// 修改getRotationError方法使用正确的目标朝向
double Payload::getRotationError() const {
    if (target_orientation_.coeffs().norm() < 0.1) {
        return 0.0;  // 如果目标朝向无效，返回0
    }
    
    Eigen::Quaterniond relative_rotation = target_orientation_ * current_orientation_.inverse();
    double rotation_error = 2.0 * std::acos(std::abs(relative_rotation.w()));
    double rotation_direction = (relative_rotation.z() >= 0) ? 1.0 : -1.0;
    double signed_rotation_error = rotation_direction * rotation_error;
    
    return signed_rotation_error;
}

Eigen::Vector2d Payload::getVelocity() const {
    if(physicsBody_) {
        b2Vec2 vel = physicsBody_->GetLinearVelocity();
        return Eigen::Vector2d(vel.x, vel.y);
    }
    return Eigen::Vector2d::Zero();
}

std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> Payload::getContactPointsAndNormals() const {
    float perimeter = 2 * (globals.PAYLOAD_HEIGHT + globals.PAYLOAD_WIDTH);
    float delta = perimeter / globals.NUM_ROBOTS;
    std::vector<Eigen::Vector2d> contact_points;
    std::vector<Eigen::Vector2d> contact_normals;

    for (int k = 0; k < globals.NUM_ROBOTS; k++) {
        float dist = k * delta;
        if ( dist < globals.PAYLOAD_WIDTH) {
            // the contact point is on the top edge

        } else if (dist < globals.PAYLOAD_WIDTH + globals.PAYLOAD_HEIGHT) {

        } else if (dist < 2 * globals.PAYLOAD_WIDTH + globals.PAYLOAD_HEIGHT) {

        } else if (dist < 2 * globals.PAYLOAD_WIDTH + 2 * globals.PAYLOAD_HEIGHT) {

        } else {
            std::cout << "Error: Contact point distance exceeds payload perimeter." << std::endl;
            return {contact_points, contact_normals};
        }
    }

    return {contact_points, contact_normals};
}

// std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> Payload::getContactPointsAndNormals() const {
//     std::vector<Eigen::Vector2d> points;
//     std::vector<Eigen::Vector2d> normals;
    
//     if (!physicsBody_) return {points, normals};
    
//     // 直接使用rotation_而不是从四元数计算
//     Eigen::Vector2d center = position_;
//     double rotation = rotation_;  // 直接使用，已经在update()中更新
    
//     // 旋转矩阵
//     Eigen::Matrix2d rot;
//     rot << cos(rotation), -sin(rotation),
//            sin(rotation),  cos(rotation);
    
//     // 在局部坐标系中定义接触点和法向量
//     // 上边的两个点
//     Eigen::Vector2d local_top1(0, -height_/2);
//     Eigen::Vector2d local_top2(0, height_/2);
//     Eigen::Vector2d local_top_normal(0, 1); // 向payload内部（向下）
    
//     // 左边的两个点
//     Eigen::Vector2d local_left1(-width_/2, 0);
//     Eigen::Vector2d local_left2(width_/2, 0);
//     Eigen::Vector2d local_left_normal(1, 0); // 向payload内部（向右）
    
//     // 转换到世界坐标系
//     points.push_back(center + rot * local_top1);
//     points.push_back(center + rot * local_top2);
//     points.push_back(center + rot * local_left1);
//     points.push_back(center + rot * local_left2);
    
//     normals.push_back(rot * local_top_normal);
//     normals.push_back(rot * local_top_normal);
//     normals.push_back(rot * local_left_normal);
//     normals.push_back(rot * local_left_normal);
    
//     return {points, normals};
// }

// 辅助函数：从四元数提取2D旋转角度
// 如果需要在其他地方使用四元数，确保getRotationFromQuaternion正确实现
double Payload::getRotationFromQuaternion(const Eigen::Quaterniond& q) const {
    // 确保四元数已经归一化
    Eigen::Quaterniond normalized_q = q.normalized();
    
    // 从四元数提取Z轴旋转角度（2D旋转）
    return atan2(2.0 * (normalized_q.w() * normalized_q.z() + normalized_q.x() * normalized_q.y()), 
                 1.0 - 2.0 * (normalized_q.y() * normalized_q.y() + normalized_q.z() * normalized_q.z()));
}