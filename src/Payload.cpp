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
    target_position_(initial_position),
    width_(width),
    height_(height),
    color_(color),
    rotation_(0.0f),
    task_completed_(false),
    target_tolerance_(2.0f),
    target_orientation_(Eigen::Quaterniond::Identity()),
    velocity_(Eigen::Vector2d::Zero()) {
        mujoco_body_id_ = -1;
        mujoco_model_ = nullptr;
        mujoco_data_ = nullptr;
        // MuJoCo body的创建将在Simulator中统一管理
        // 这里不直接创建，而是等待Simulator分配ID
    }
Payload::~Payload(){
    // MuJoCo body的销毁由Simulator统一管理
    // 这里不需要手动清理
}

void Payload::setMuJoCoReferences(mjModel* model, mjData* data) {
    mujoco_model_ = model;
    mujoco_data_ = data;
}

void Payload::createMuJoCoBody(float density) {
    // 这个方法现在只是占位，实际的body创建在Simulator中进行
    if (mujoco_body_id_ < 0) {
        std::cerr << "Warning: Payload " << payload_id_ << " MuJoCo body ID not set" << std::endl;
    }
}

void Payload::syncFromMuJoCo() {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) return;
    
    int pos_adr = mujoco_model_->jnt_qposadr[mujoco_body_id_];
    int vel_adr = mujoco_model_->jnt_dofadr[mujoco_body_id_];
    
    // 更新位置
    position_.x() = mujoco_data_->qpos[pos_adr + 0];
    position_.y() = mujoco_data_->qpos[pos_adr + 1];
    
    // 更新四元数朝向
    current_orientation_.w() = mujoco_data_->qpos[pos_adr + 3];
    current_orientation_.x() = mujoco_data_->qpos[pos_adr + 4];
    current_orientation_.y() = mujoco_data_->qpos[pos_adr + 5];
    current_orientation_.z() = mujoco_data_->qpos[pos_adr + 6];
    
    // 更新速度
    velocity_.x() = mujoco_data_->qvel[vel_adr + 0];
    velocity_.y() = mujoco_data_->qvel[vel_adr + 1];
    current_angular_velocity_ = mujoco_data_->qvel[vel_adr + 5];  // 绕z轴角速度
    
    // 更新2D旋转角度
    rotation_ = getRotationFromQuaternion(current_orientation_);
}

void Payload::syncToMuJoCo() {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) return;
    
    int pos_adr = mujoco_model_->jnt_qposadr[mujoco_body_id_];
    int vel_adr = mujoco_model_->jnt_dofadr[mujoco_body_id_];
    
    // 同步位置和朝向
    mujoco_data_->qpos[pos_adr + 0] = position_.x();
    mujoco_data_->qpos[pos_adr + 1] = position_.y();
    mujoco_data_->qpos[pos_adr + 2] = 0.5;              // payload高度
    
    // 同步四元数朝向
    mujoco_data_->qpos[pos_adr + 3] = current_orientation_.w();
    mujoco_data_->qpos[pos_adr + 4] = current_orientation_.x();
    mujoco_data_->qpos[pos_adr + 5] = current_orientation_.y();
    mujoco_data_->qpos[pos_adr + 6] = current_orientation_.z();
    
    // 同步速度
    mujoco_data_->qvel[vel_adr + 0] = velocity_.x();
    mujoco_data_->qvel[vel_adr + 1] = velocity_.y();
    mujoco_data_->qvel[vel_adr + 2] = 0.0;              // vz
    mujoco_data_->qvel[vel_adr + 3] = 0.0;              // wx
    mujoco_data_->qvel[vel_adr + 4] = 0.0;              // wy
    mujoco_data_->qvel[vel_adr + 5] = current_angular_velocity_;  // wz
}

Eigen::Vector2d Payload::getMuJoCoPosition() const {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) {
        return position_;
    }
    
    int pos_adr = mujoco_model_->jnt_qposadr[mujoco_body_id_];
    return Eigen::Vector2d(mujoco_data_->qpos[pos_adr + 0], mujoco_data_->qpos[pos_adr + 1]);
}

Eigen::Vector2d Payload::getMuJoCoVelocity() const {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) {
        return velocity_;
    }
    
    int vel_adr = mujoco_model_->jnt_dofadr[mujoco_body_id_];
    return Eigen::Vector2d(mujoco_data_->qvel[vel_adr + 0], mujoco_data_->qvel[vel_adr + 1]);
}

double Payload::getMuJoCoRotation() const {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) {
        return rotation_;
    }
    
    int pos_adr = mujoco_model_->jnt_qposadr[mujoco_body_id_];
    Eigen::Quaterniond quat(
        mujoco_data_->qpos[pos_adr + 3],  // w
        mujoco_data_->qpos[pos_adr + 4],  // x
        mujoco_data_->qpos[pos_adr + 5],  // y
        mujoco_data_->qpos[pos_adr + 6]   // z
    );
    return getRotationFromQuaternion(quat);
}

double Payload::getMuJoCoAngularVelocity() const {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) {
        return current_angular_velocity_;
    }
    
    int vel_adr = mujoco_model_->jnt_dofadr[mujoco_body_id_];
    return mujoco_data_->qvel[vel_adr + 5];  // wz分量
}

double Payload::getMuJoCoMass() const {
    if (mujoco_body_id_ < 0 || !mujoco_model_) {
        return width_ * height_ * globals.PAYLOAD_DENSITY;  // 估算
    }
    
    return mujoco_model_->body_mass[mujoco_body_id_];
}

double Payload::getMuJoCoInertia() const {
    if (mujoco_body_id_ < 0 || !mujoco_model_) {
        // 矩形的转动惯量估算
        double mass = width_ * height_ * globals.PAYLOAD_DENSITY;
        return mass * (width_ * width_ + height_ * height_) / 12.0;
    }
    
    // MuJoCo中的转动惯量（绕z轴）
    return mujoco_model_->body_inertia[mujoco_body_id_ * 3 + 2];
}

// 7. 重写状态设置方法
void Payload::setMuJoCoPosition(const Eigen::Vector2d& position) {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) {
        position_ = position;
        return;
    }
    
    int pos_adr = mujoco_model_->jnt_qposadr[mujoco_body_id_];
    mujoco_data_->qpos[pos_adr + 0] = position.x();
    mujoco_data_->qpos[pos_adr + 1] = position.y();
}

void Payload::setMuJoCoVelocity(const Eigen::Vector2d& velocity) {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) {
        velocity_ = velocity;
        return;
    }
    
    int vel_adr = mujoco_model_->jnt_dofadr[mujoco_body_id_];
    mujoco_data_->qvel[vel_adr + 0] = velocity.x();
    mujoco_data_->qvel[vel_adr + 1] = velocity.y();
}

void Payload::setMuJoCoRotation(double rotation) {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) {
        rotation_ = rotation;
        current_orientation_ = Eigen::Quaterniond(cos(rotation/2), 0, 0, sin(rotation/2));
        return;
    }
    
    int pos_adr = mujoco_model_->jnt_qposadr[mujoco_body_id_];
    Eigen::Quaterniond quat(cos(rotation/2), 0, 0, sin(rotation/2));
    
    mujoco_data_->qpos[pos_adr + 3] = quat.w();
    mujoco_data_->qpos[pos_adr + 4] = quat.x();
    mujoco_data_->qpos[pos_adr + 5] = quat.y();
    mujoco_data_->qpos[pos_adr + 6] = quat.z();
}

void Payload::setMuJoCoAngularVelocity(double angular_velocity) {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) {
        current_angular_velocity_ = angular_velocity;
        return;
    }
    
    int vel_adr = mujoco_model_->jnt_dofadr[mujoco_body_id_];
    mujoco_data_->qvel[vel_adr + 5] = angular_velocity;
}

// 8. 重写力控制方法
void Payload::applyMuJoCoForce(const Eigen::Vector2d& force, const Eigen::Vector2d& point) {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) return;
    
    if (point.norm() < 1e-6) {
        // 在质心施加力
        mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 0] = force.x();
        mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 1] = force.y();
        mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 2] = 0.0;
        mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 3] = 0.0;
        mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 4] = 0.0;
        mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 5] = 0.0;
    } else {
        // 在指定点施加力
        Eigen::Vector2d center = getMuJoCoPosition();
        Eigen::Vector2d r = point - center;
        double torque = r.x() * force.y() - r.y() * force.x();
        
        mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 0] = force.x();
        mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 1] = force.y();
        mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 2] = 0.0;
        mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 3] = 0.0;
        mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 4] = 0.0;
        mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 5] = torque;
    }
}

void Payload::applyMuJoCoTorque(double torque) {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) return;
    
    mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 5] = torque;
}



void Payload::setTargetFromRelativeRotation(double relative_rotation_rad) {
    // target_orientation = initial_orientation * relative_rotation
    Eigen::Quaterniond relative_rotation(cos(relative_rotation_rad/2), 0, 0, sin(relative_rotation_rad/2));
    target_orientation_ = initial_orientation_ * relative_rotation;
    
    std::cout << "Target orientation set: initial + " << relative_rotation_rad 
              << " rad = " << getRotationFromQuaternion(target_orientation_) << " rad" << std::endl;
}

void Payload::update() {
    // 从MuJoCo同步状态
    syncFromMuJoCo();
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

void Payload::draw() {
    float x = static_cast<float>(position_.x());
    float y = static_cast<float>(position_.y());
    Vector3 position3D = {x, 0.5f, y};
    Vector3 size = {width_, 1.0f, height_};
    
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
            static_cast<float>(target_center.x + arrow_length * std::cos(target_angle)),
            target_center.y,
            static_cast<float>(target_center.z + arrow_length * std::sin(target_angle))
        };
        
        // 绘制朝向箭头
        DrawLine3D(target_center, arrow_end, ORANGE);
        DrawSphere(arrow_end, 0.8f, ORANGE);  // 箭头头部
        
        // 可选：绘制朝向标识文字
        // DrawText3D(font, "TARGET", arrow_end, 1.0f, 1.0f, false, WHITE);
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


Eigen::Vector2d Payload::getPosition() const {
    return getMuJoCoPosition();
}

float Payload::getMass() const {
    return static_cast<float>(getMuJoCoMass());
}

double Payload::getMomentOfInertia() const {
    return getMuJoCoInertia();
}

double Payload::getAngularVelocity() const {
    return getMuJoCoAngularVelocity();
}

Eigen::Quaterniond Payload::getRotation() const{
    return current_orientation_;
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
    return getMuJoCoVelocity();
}

std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> Payload::getContactPointsAndNormals() const {
    std::vector<Eigen::Vector2d> points;
    std::vector<Eigen::Vector2d> normals;
    
    // 直接使用rotation_而不是从四元数计算
    Eigen::Vector2d center = position_;
    double rotation = rotation_;  // 直接使用，已经在update()中更新
    
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
// 如果需要在其他地方使用四元数，确保getRotationFromQuaternion正确实现
double Payload::getRotationFromQuaternion(const Eigen::Quaterniond& q) const {
    // 确保四元数已经归一化
    Eigen::Quaterniond normalized_q = q.normalized();
    
    // 从四元数提取Z轴旋转角度（2D旋转）
    return atan2(2.0 * (normalized_q.w() * normalized_q.z() + normalized_q.x() * normalized_q.y()), 
                 1.0 - 2.0 * (normalized_q.y() * normalized_q.y() + normalized_q.z() * normalized_q.z()));
}