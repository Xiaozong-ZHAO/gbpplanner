/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <Robot.h>
#include <Simulator.h>

/***************************************************************************/
// Creates a robot. Inputs required are :
//      - Pointer to the simulator
//      - A robot id rid (should be taken from simulator->next_rid_++),
//      - A dequeue of waypoints (which are 4 dimensional [x,y,xdot,ydot])
//      - Robot radius
//      - Colour
/***************************************************************************/
Robot::Robot(Simulator* sim,
             int rid,
             std::deque<Eigen::VectorXd> waypoints,
             float size,
             Color color) : FactorGraph{rid},
             sim_(sim), rid_(rid),
             waypoints_(waypoints),
             robot_radius_(size), color_(color) {


    height_3D_ = robot_radius_;     // Height out of plane for 3d visualisation only
    mujoco_body_id_ = -1;  // 无效ID，将在后续设置
    mujoco_joint_id_ = -1;
    mujoco_model_ = nullptr;
    mujoco_data_ = nullptr;


    // Robot will always set its horizon state to move towards the next waypoint.
    // Once this waypoint has been reached, it pops it from the waypoints
    Eigen::VectorXd start = position_ = waypoints_[0];
    waypoints_.pop_front();                             
    auto goal = (waypoints_.size()>0) ? waypoints_[0] : start;


    // Initialise the horzion in the direction of the goal, at a distance T_HORIZON * MAX_SPEED from the start.
    Eigen::VectorXd start2goal = goal - start;
    Eigen::VectorXd horizon = start + std::min(start2goal.norm(), 1.*globals.T_HORIZON*globals.MAX_SPEED)*start2goal.normalized();

    // Variables representing the planned path are at timesteps which increase in spacing.
    // eg. (so that a span of 10 timesteps as a planning horizon can be represented by much fewer variables)
    std::vector<int> variable_timesteps = getVariableTimesteps(globals.T_HORIZON / globals.T0, globals.LOOKAHEAD_MULTIPLE);
    num_variables_ = variable_timesteps.size();

    /***************************************************************************/
    /* Create Variables with fixed pose priors on start and horizon variables. */
    /***************************************************************************/
    Color var_color = color_; double sigma; int n = globals.N_DOFS;
    Eigen::VectorXd mu(n); Eigen::VectorXd sigma_list(n); 
    for (int i = 0; i < num_variables_; i++){
        // Set initial mu and covariance of variable interpolated between start and horizon
        mu = start + (horizon - start) * (float)(variable_timesteps[i]/(float)variable_timesteps.back());
        // Start and Horizon state variables should be 'fixed' during optimisation at a timestep
        sigma = (i==0 || i==num_variables_-1) ? globals.SIGMA_POSE_FIXED : 0.;
        sigma_list.setConstant(sigma);
        
        // Create variable and add to robot's factor graph 
        auto variable = std::make_shared<Variable>(sim->next_vid_++, rid_, mu, sigma_list, robot_radius_, n);
        variables_[variable->key_] = variable;
    }

    /***************************************************************************/
    /* Create Dynamics factors between variables */
    /***************************************************************************/
    for (int i = 0; i < num_variables_-1; i++)
    {
        // T0 is the timestep between the current state and the first planned state.
        float delta_t = globals.T0 * (variable_timesteps[i + 1] - variable_timesteps[i]);
        std::vector<std::shared_ptr<Variable>> variables {getVar(i), getVar(i+1)};
        auto factor = std::make_shared<DynamicsFactor>(sim->next_fid_++, rid_, variables, globals.SIGMA_FACTOR_DYNAMICS, Eigen::VectorXd::Zero(globals.N_DOFS), delta_t);
        
        // Add this factor to the variable's list of adjacent factors, as well as to the robot's list of factors
        for (auto var : factor->variables_) var->add_factor(factor);
        factors_[factor->key_] = factor;
    }

    /***************************************************************************/
    // Create Obstacle factors for all variables excluding start, excluding horizon
    /***************************************************************************/
    for (int i = 1; i < num_variables_-1; i++)
    {
        std::vector<std::shared_ptr<Variable>> variables{getVar(i)};
        auto fac_obs = std::make_shared<ObstacleFactor>(sim, sim->next_fid_++, rid_, variables, globals.SIGMA_FACTOR_OBSTACLE, Eigen::VectorXd::Zero(1), &(sim_->obstacleImg));

        // Add this factor to the variable's list of adjacent factors, as well as to the robot's list of factors
        for (auto var : fac_obs->variables_) var->add_factor(fac_obs);
        this->factors_[fac_obs->key_] = fac_obs;
    }

    payload_twist_variables_.clear();  // 初始化容器
    payload_geometry_cached_ = false;

    for (int i = 1; i < num_variables_ - 1; i++){
        std::vector<std::shared_ptr<Variable>> variables{getVar(i)};
        auto fac_force = std::make_shared<ForceAllocationFactor>(sim, sim->next_fid_++, rid_, variables, globals.SIGMA_FACTOR_FORCE_ALLOCATION, Eigen::VectorXd::Zero(3));

        for (auto var : fac_force->variables_) var->add_factor(fac_force);
        this->factors_[fac_force->key_] = fac_force;
    }

};

// 7. 重写物理属性查询方法
Eigen::Vector2d Robot::getMuJoCoPosition() const {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) {
        return Eigen::Vector2d(position_(0), position_(1));
    }
    
    int pos_adr = mujoco_model_->jnt_qposadr[mujoco_body_id_];
    return Eigen::Vector2d(mujoco_data_->qpos[pos_adr + 0], mujoco_data_->qpos[pos_adr + 1]);
}

Eigen::Vector2d Robot::getMuJoCoVelocity() const {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) {
        if (position_.size() >= 4) {
            return Eigen::Vector2d(position_(2), position_(3));
        }
        return Eigen::Vector2d::Zero();
    }
    
    int vel_adr = mujoco_model_->jnt_dofadr[mujoco_body_id_];
    return Eigen::Vector2d(mujoco_data_->qvel[vel_adr + 0], mujoco_data_->qvel[vel_adr + 1]);
}

void Robot::setMuJoCoPosition(const Eigen::Vector2d& position) {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) return;
    
    int pos_adr = mujoco_model_->jnt_qposadr[mujoco_body_id_];
    mujoco_data_->qpos[pos_adr + 0] = position.x();
    mujoco_data_->qpos[pos_adr + 1] = position.y();
}

void Robot::setMuJoCoVelocity(const Eigen::Vector2d& velocity) {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) return;
    
    int vel_adr = mujoco_model_->jnt_dofadr[mujoco_body_id_];
    mujoco_data_->qvel[vel_adr + 0] = velocity.x();
    mujoco_data_->qvel[vel_adr + 1] = velocity.y();
}

void Robot::syncToMuJoCo() {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) return;
    
    int pos_adr = mujoco_model_->jnt_qposadr[mujoco_body_id_];
    int vel_adr = mujoco_model_->jnt_dofadr[mujoco_body_id_];
    
    // 设置位置
    mujoco_data_->qpos[pos_adr + 0] = position_(0);     // x
    mujoco_data_->qpos[pos_adr + 1] = position_(1);     // y
    mujoco_data_->qpos[pos_adr + 2] = 0.1;              // z (固定高度)
    
    // 设置朝向为无旋转（四元数）
    mujoco_data_->qpos[pos_adr + 3] = 1.0;              // qw
    mujoco_data_->qpos[pos_adr + 4] = 0.0;              // qx
    mujoco_data_->qpos[pos_adr + 5] = 0.0;              // qy
    mujoco_data_->qpos[pos_adr + 6] = 0.0;              // qz
    
    // 设置速度
    if (position_.size() >= 4) {
        mujoco_data_->qvel[vel_adr + 0] = position_(2); // vx
        mujoco_data_->qvel[vel_adr + 1] = position_(3); // vy
        mujoco_data_->qvel[vel_adr + 2] = 0.0;          // vz
        mujoco_data_->qvel[vel_adr + 3] = 0.0;          // wx
        mujoco_data_->qvel[vel_adr + 4] = 0.0;          // wy
        mujoco_data_->qvel[vel_adr + 5] = 0.0;          // wz
    }
}

void Robot::applyMuJoCoForce(const Eigen::Vector2d& force, const Eigen::Vector2d& point) {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) return;
    
    // 使用MuJoCo的external force接口
    if (point.norm() < 1e-6) {
        // 在质心施加力
        mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 0] = force.x();
        mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 1] = force.y();
        mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 2] = 0.0;
    } else {
        // 在指定点施加力（需要计算等效的力和力矩）
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

void Robot::applyMuJoCoTorque(double torque) {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) return;
    
    mujoco_data_->xfrc_applied[mujoco_body_id_ * 6 + 5] = torque;
}



void Robot::syncFromMuJoCo() {
    if (mujoco_body_id_ < 0 || !mujoco_model_ || !mujoco_data_) return;
    
    // 计算在qpos和qvel中的索引
    int pos_adr = mujoco_model_->jnt_qposadr[mujoco_body_id_];
    int vel_adr = mujoco_model_->jnt_dofadr[mujoco_body_id_];
    
    // 更新位置 (假设free joint: x, y, z, qw, qx, qy, qz)
    position_(0) = mujoco_data_->qpos[pos_adr + 0];     // x
    position_(1) = mujoco_data_->qpos[pos_adr + 1];     // y
    
    // 更新速度 (假设free joint: vx, vy, vz, wx, wy, wz)
    if (position_.size() >= 4) {
        position_(2) = mujoco_data_->qvel[vel_adr + 0]; // vx
        position_(3) = mujoco_data_->qvel[vel_adr + 1]; // vy
    }
}

/***************************************************************************************************/
/* Destructor */
/***************************************************************************************************/
Robot::~Robot(){
    detachFromPayloadMuJoCo();
}

void Robot::setMuJoCoReferences(mjModel* model, mjData* data) {
    mujoco_model_ = model;
    mujoco_data_ = data;
}

void Robot::createContactForceVariables() {
    // 为每个时间步创建接触力变量（标量，表示法向接触力大小）
    for (int i = 0; i < num_variables_; i++) {
        Eigen::VectorXd force_mu = Eigen::VectorXd::Zero(1);  // 初始接触力为0
        Eigen::VectorXd force_sigma = Eigen::VectorXd::Constant(1, 1.0);  // 适中的先验强度
        
        auto contact_force_var = std::make_shared<Variable>(
            sim_->next_vid_++, rid_, 
            force_mu, force_sigma, 
            robot_radius_, 1  // 1DOF for contact force magnitude
        );
        
        variables_[contact_force_var->key_] = contact_force_var;
        contact_force_variables_.push_back(contact_force_var);
    }
    
    std::cout << "Robot " << rid_ << " created " << contact_force_variables_.size() 
              << " contact force variables" << std::endl;
}

void Robot::updateContactForceGeometry() {
    if (!force_allocation_factor_) return;
    
    // 更新力分配因子的几何参数
    if (!sim_->payloads_.empty()) {
        auto payload = sim_->payloads_.begin()->second;
        auto [contact_points, contact_normals] = payload->getContactPointsAndNormals();
        force_allocation_factor_->updateGeometry(contact_points, contact_normals);
    }
}

void Robot::cachePayloadGeometry() {
    if (payload_geometry_cached_ || sim_->payloads_.empty()) return;
    
    auto payload = sim_->payloads_.begin()->second;
    auto [contact_points, contact_normals] = payload->getContactPointsAndNormals();
    
    if (assigned_contact_point_index_ >= contact_points.size()) {
        std::cout << "Warning: Invalid contact point index for robot " << rid_ << std::endl;
        return;
    }
    
    // 缓存几何参数
    Eigen::Vector2d contact_point = contact_points[assigned_contact_point_index_];
    Eigen::Vector2d payload_center = payload->getPosition();
    
    cached_r_vector_ = contact_point - payload_center;
    cached_contact_normal_ = contact_normals[assigned_contact_point_index_];
    payload_geometry_cached_ = true;
    
}

void Robot::updatePayloadFactorGeometry() {
    if (payload_twist_factors_.empty()) return;
    
    // 首先缓存几何参数
    cachePayloadGeometry();
    if (!payload_geometry_cached_) return;
    
    // 更新所有PayloadTwistFactor的几何参数
    for (auto factor : payload_twist_factors_) {
        factor->updateGeometry(cached_r_vector_, cached_contact_normal_);
    }
}

std::vector<Eigen::Vector3d> Robot::computeDesiredPayloadTwists() {
    std::vector<Eigen::Vector3d> desired_twists;
    
    if (sim_->payloads_.empty() || payload_twist_variables_.empty()) {
        return std::vector<Eigen::Vector3d>(num_variables_, Eigen::Vector3d::Zero());
    }
    
    auto payload = sim_->payloads_.begin()->second;
    
    // 计算当前和目标twist
    Eigen::Vector3d current_twist = Eigen::Vector3d::Zero();
    Eigen::Vector2d current_velocity = payload->getVelocity();
    current_twist(0) = current_velocity.x();
    current_twist(1) = current_velocity.y();
    current_twist(2) = payload->getAngularVelocity();
    
    // 计算目标twist
    Eigen::Vector3d target_twist = Eigen::Vector3d::Zero();
    
    // 线速度
    Eigen::Vector2d payload_to_target = payload->target_position_ - payload->position_;
    if (payload_to_target.norm() > 0.1) {
        Eigen::Vector2d desired_velocity = payload_to_target.normalized() * globals.MAX_SPEED * 0.5;
        target_twist(0) = desired_velocity.x();
        target_twist(1) = desired_velocity.y();
    }
    
    // 角速度
    double rotation_error = payload->getRotationError();
    if (std::abs(rotation_error) > 0.01) {
        double desired_angular_velocity = std::copysign(1.0, rotation_error) * 
            std::min(static_cast<double>(globals.MAX_ANGULAR_SPEED * 0.5), std::abs(rotation_error));
        target_twist(2) = desired_angular_velocity;
    }
    
    // 为每个时间步插值计算期望twist
    for (int i = 0; i < num_variables_; i++) {
        Eigen::Vector3d interpolated_twist = interpolatePayloadTwist(i, current_twist, target_twist);
        desired_twists.push_back(interpolated_twist);
    }
    
    return desired_twists;
}

Eigen::Vector3d Robot::interpolatePayloadTwist(int var_index, 
                                              const Eigen::Vector3d& current_twist,
                                              const Eigen::Vector3d& target_twist) {
    if (var_index == 0) {
        // 当前状态：使用当前twist
        return current_twist;
    } else if (var_index == num_variables_ - 1) {
        // 地平线状态：使用目标twist
        return target_twist;
    } else {
        // 中间状态：线性插值
        double alpha = (double)var_index / (double)(num_variables_ - 1);
        return (1.0 - alpha) * current_twist + alpha * target_twist;
    }
}

void Robot::updatePayloadTwistPriors() {
    if (payload_twist_variables_.empty()) return;
    
    // 计算所有时间步的期望twist
    std::vector<Eigen::Vector3d> desired_twists = computeDesiredPayloadTwists();
    
    // 更新每个payload variable的先验
    for (int i = 0; i < payload_twist_variables_.size() && i < desired_twists.size(); i++) {
        payload_twist_variables_[i]->change_variable_prior(desired_twists[i]);
    }
}

// Robot.cpp 中的正确实现
void Robot::attachToPayloadMuJoCo(std::shared_ptr<Payload> payload, const Eigen::Vector2d& attach_point) {
    if (mujoco_body_id_ < 0 || payload->mujoco_body_id_ < 0) {
        std::cout << "Error: Cannot attach - invalid MuJoCo body IDs" << std::endl;
        return;
    }
    
    if (mujoco_joint_id_ >= 0) {
        detachFromPayloadMuJoCo(); // 先断开现有连接
    }
    
    // 在MuJoCo中创建约束需要修改模型，这比较复杂
    // 简化方案：使用equality constraints运行时创建连接
    if (!mujoco_model_ || !mujoco_data_) return;
    
    // 检查是否有可用的equality constraint slots
    if (mujoco_data_->ne >= mujoco_model_->neq) {
        std::cout << "Error: No available equality constraints for attachment" << std::endl;
        return;
    }
    
    // 创建位置约束（将在下一个mj_step中生效）
    // 这是简化版本，实际上可能需要更复杂的约束设置
    mujoco_joint_id_ = mujoco_data_->ne;  // 使用下一个可用约束ID
    
    // 注意：完整的关节创建需要在XML模型中预定义或使用MuJoCo的动态约束API
    std::cout << "Robot " << rid_ << " attached to payload using MuJoCo constraint " 
              << mujoco_joint_id_ << std::endl;
}

void Robot::attachToPayload(std::shared_ptr<Payload> payload, const Eigen::Vector2d& attach_point) {
    // 调用MuJoCo版本的实现
    attachToPayloadMuJoCo(payload, attach_point);
}

void Robot::detachFromPayloadMuJoCo() {
    if (mujoco_joint_id_ >= 0) {
        // 禁用对应的equality constraint
        if (mujoco_data_ && mujoco_joint_id_ < mujoco_data_->ne) {
            // 设置约束为非活动状态（具体实现依赖于MuJoCo版本）
            // mujoco_data_->eq_active[mujoco_joint_id_] = 0;  // 如果支持的话
        }
        
        mujoco_joint_id_ = -1;
        std::cout << "Robot " << rid_ << " detached from payload" << std::endl;
    }
}


void Robot::deletePayloadTwistFactors(int payload_id) {
    std::vector<Key> factors_to_delete{};
    
    // 查找所有 PayloadTwistFactor（注意：现在没有 getPayloadId() 方法了）
    for (auto& [f_key, fac] : this->factors_) {
        if (fac->factor_type_ == PAYLOAD_TWIST_FACTOR) {
            // 由于删除了 payload 指针，这里需要其他方式识别
            // 可以通过变量连接来判断，或者添加 payload_id 成员
            factors_to_delete.push_back(f_key);
        }
    }
    
    // 删除因子
    for (auto f_key : factors_to_delete) {
        auto fac = this->factors_[f_key];
        for (auto& var : fac->variables_) {
            var->delete_factor(f_key);
        }
        this->factors_.erase(f_key);
    }
    
    // std::cout << "Deleted " << factors_to_delete.size() 
    //           << " PayloadTwistFactors for payload " << payload_id << std::endl;
}

bool Robot::isConnectedToPayload(int payload_id) const {
    return std::find(connected_payload_ids_.begin(), connected_payload_ids_.end(), payload_id) 
           != connected_payload_ids_.end();
}


void Robot::createMuJoCoBody() {
    if (mujoco_body_id_ > 0){
        std::cerr << "Warning: Robot" << rid_ << "MuJoCo body ID not set" <<std::endl;
    }
}

/***************************************************************************************************/
/* Change the prior of the Current state */
/***************************************************************************************************/

void updateForPayload(){
    
}

void Robot::updateCurrent(){
    // Move plan: move plan current state by plan increment
    Eigen::VectorXd increment = ((*this)[1]->mu_ - (*this)[0]->mu_) * globals.TIMESTEP / globals.T0;
    // In GBP we do this by modifying the prior on the variable
    getVar(0)->change_variable_prior(getVar(0)->mu_ + increment);
    // Real pose update
    position_ = position_ + increment;
    
    syncToMuJoCo();
};

void Robot::syncLogicalToPhysics(){
    syncToMuJoCo();
}

void Robot::syncPhysicsToLogical(){
    syncFromMuJoCo();
}

/***************************************************************************************************/
/* Change the prior of the Horizon state */
/***************************************************************************************************/
void Robot::updateHorizon(){
    // Horizon state moves towards the next waypoint.
    // The Horizon state's velocity is capped at MAX_SPEED
    auto horizon = getVar(14);      // get horizon state variable
    Eigen::VectorXd dist_horz_to_goal = waypoints_.front()({0,1}) - horizon->mu_({0,1});                        
    Eigen::VectorXd new_vel = dist_horz_to_goal.normalized() * std::min((double)globals.MAX_SPEED, dist_horz_to_goal.norm());
    Eigen::VectorXd new_pos = horizon->mu_({0,1}) + new_vel*globals.TIMESTEP;

    horizon->mu_ << new_pos, new_vel;
    horizon->change_variable_prior(horizon->mu_);

    // If the horizon has reached the waypoint, pop that waypoint from the waypoints.
    // Could add other waypoint behaviours here (maybe they might move, or change randomly).
    if (dist_horz_to_goal.norm() < robot_radius_){
        if (waypoints_.size()>1) waypoints_.pop_front();
    }
}

/***************************************************************************************************/
// For new neighbours of a robot, create inter-robot factors if they don't exist. 
// Delete existing inter-robot factors for faraway robots
/***************************************************************************************************/
void Robot::updateInterrobotFactors(){
    
    // Search through currently connected rids. If any are not in neighbours, delete interrobot factors.
    for (auto rid : connected_r_ids_){
        if (std::find(neighbours_.begin(), neighbours_.end(), rid)==neighbours_.end()){
            deleteInterrobotFactors(sim_->robots_.at(rid));
        };
    }
    // Search through neighbours. If any are not in currently connected rids, create interrobot factors.
    for (auto rid : neighbours_){
        if (std::find(connected_r_ids_.begin(), connected_r_ids_.end(), rid)==connected_r_ids_.end()){
            createInterrobotFactors(sim_->robots_.at(rid));
            if (!sim_->symmetric_factors) sim_->robots_.at(rid)->connected_r_ids_.push_back(rid_);
        };
    }
}

/***************************************************************************************************/
// Create inter-robot factors between this robot and another robot
/***************************************************************************************************/
void Robot::createInterrobotFactors(std::shared_ptr<Robot> other_robot)
{
    // Create Interrobot factors for all timesteps excluding current state
    for (int i = 1; i < num_variables_; i++){
        // Get variables
        std::vector<std::shared_ptr<Variable>> variables{getVar(i), other_robot->getVar(i)};

        // Create the inter-robot factor
        Eigen::VectorXd z = Eigen::VectorXd::Zero(variables.front()->n_dofs_);
        auto factor = std::make_shared<InterrobotFactor>(sim_->next_fid_++, this->rid_, variables, globals.SIGMA_FACTOR_INTERROBOT, z, 0.5*(this->robot_radius_ + other_robot->robot_radius_));
        factor->other_rid_ = other_robot->rid_;
        // Add factor the the variable's list of factors, as well as to the robot's list of factors
        for (auto var : factor->variables_) var->add_factor(factor);
        this->factors_[factor->key_] = factor;
    }

    // Add the other robot to this robot's list of connected robots.
    this->connected_r_ids_.push_back(other_robot->rid_);
};

/***************************************************************************************************/
/* Delete interrobot factors between the two robots */
/***************************************************************************************************/
void Robot::deleteInterrobotFactors(std::shared_ptr<Robot> other_robot)
{
    std::vector<Key> facs_to_delete{};
    for (auto& [f_key, fac] : this->factors_){
        if (fac->other_rid_ != other_robot->rid_) continue;

        // Only get here if factor is connected to a variable in the other_robot
        for (auto& var : fac->variables_){ 
            var->delete_factor(f_key);
            facs_to_delete.push_back(f_key);
        }
    }
    for (auto f_key : facs_to_delete) this->factors_.erase(f_key);

    // Remove other robot from current robot's connected rids
    auto it = std::find(connected_r_ids_.begin(), connected_r_ids_.end(), other_robot->rid_);
    if (it != connected_r_ids_.end()){
        connected_r_ids_.erase(it);
    }

};

/***************************************************************************************************/
// Drawing functions for the robot.
// We deal with a 2d problem, so the out-of-plane height is set to height_3D_.
/***************************************************************************************************/
void Robot::draw(){
    Color col = (interrobot_comms_active_) ? color_ : GRAY;
    
    // 现有绘制代码保持不变...
    if (globals.DRAW_PATH){
        static int debug = 0;
        for (auto [vid, variable] : variables_){
            if (!variable->valid_) continue;
            DrawSphere(Vector3{(float)variable->mu_(0), height_3D_, (float)variable->mu_(1)}, 0.5*robot_radius_, ColorAlpha(col, 0.5));
        }
        for (auto [fid, factor] : factors_) factor->draw();
    }
    
    // 现有连接线绘制...
    if (globals.DRAW_INTERROBOT){
        for (auto rid : connected_r_ids_){
            if (!interrobot_comms_active_ || !sim_->robots_.at(rid)->interrobot_comms_active_) continue;
            DrawCylinderEx(Vector3{(float)position_(0), height_3D_, (float)position_(1)},
                            Vector3{(float)(*sim_->robots_.at(rid))[0]->mu_(0), sim_->robots_.at(rid)->height_3D_, (float)(*sim_->robots_.at(rid))[0]->mu_(1)}, 
                            0.1, 0.1, 4, BLACK);
        }
    }

    // 现有waypoints绘制...
    if (globals.DRAW_WAYPOINTS){
        for (int wp_idx=0; wp_idx<waypoints_.size(); wp_idx++){
            DrawCubeV(Vector3{(float)waypoints_[wp_idx](0), height_3D_, (float)waypoints_[wp_idx](1)}, Vector3{1.f*robot_radius_,1.f*robot_radius_,1.f*robot_radius_}, col);
        }
    }
    
    // 新增：绘制速度向量
    if (globals.DRAW_ROBOT_VELOCITIES) {
        drawVelocityVector();
    }
    
    // 现有机器人模型绘制...
    DrawModel(sim_->graphics->robotModel_, Vector3{(float)position_(0), height_3D_, (float)position_(1)}, robot_radius_, col);
}

void Robot::drawVelocityVector() {
    // 获取当前速度和期望速度
    Eigen::Vector2d current_velocity = getCurrentVelocity();
    Eigen::Vector2d desired_velocity = getDesiredVelocity();
    
    Vector3 robot_pos = {(float)position_(0), height_3D_ + 1.0f, (float)position_(1)};
    
    // 速度缩放因子（调整箭头长度）
    float velocity_scale = 5.0f;
    float min_velocity_threshold = 0.01f; // 最小速度阈值，避免绘制过小的向量
    
    // 绘制当前速度向量（绿色）
    if (current_velocity.norm() > min_velocity_threshold) {
        Vector3 current_vel_end = {
            robot_pos.x + static_cast<float>(current_velocity.x() * velocity_scale),
            robot_pos.y,
            robot_pos.z + static_cast<float>(current_velocity.y() * velocity_scale)
        };
        
        // 绘制速度箭头
        DrawLine3D(robot_pos, current_vel_end, GREEN);
        DrawSphere(current_vel_end, 0.3f, GREEN);
        
        // 可选：绘制速度数值文本
        char vel_text[64];
        sprintf(vel_text, "%.2f", current_velocity.norm());
        Vector3 text_pos = {current_vel_end.x, current_vel_end.y + 0.5f, current_vel_end.z};
        // DrawText3D(font, vel_text, text_pos, 1.0f, 1.0f, false, GREEN); // 需要font支持
    }
    
    // 绘制期望速度向量（蓝色）
    if (desired_velocity.norm() > min_velocity_threshold) {
        Vector3 desired_vel_end = {
            robot_pos.x + static_cast<float>(desired_velocity.x() * velocity_scale),
            robot_pos.y + 0.2f, // 稍微偏移避免重叠
            robot_pos.z + static_cast<float>(desired_velocity.y() * velocity_scale)
        };
        
        // 绘制期望速度箭头
        DrawLine3D(robot_pos, desired_vel_end, BLUE);
        DrawSphere(desired_vel_end, 0.25f, BLUE);
    }
    
    // 绘制速度误差向量（红色）
    Eigen::Vector2d velocity_error = desired_velocity - current_velocity;
    if (velocity_error.norm() > min_velocity_threshold) {
        Vector3 error_end = {
            robot_pos.x + static_cast<float>(velocity_error.x() * velocity_scale),
            robot_pos.y - 0.2f, // 向下偏移
            robot_pos.z + static_cast<float>(velocity_error.y() * velocity_scale)
        };
        
        DrawLine3D(robot_pos, error_end, RED);
        DrawSphere(error_end, 0.2f, RED);
    }
}

Eigen::Vector2d Robot::getCurrentVelocity() const {
    // 优先从MuJoCo获取
    if (mujoco_body_id_ >= 0 && mujoco_model_ && mujoco_data_) {
        return getMuJoCoVelocity();
    }
    
    // Fallback到原有逻辑
    if (variables_.size() >= 2) {
        auto current_var = getVar(0);
        auto next_var = getVar(1);
        if (current_var && next_var && current_var->valid_ && next_var->valid_) {
            Eigen::Vector2d pos_diff = next_var->mu_.head(2) - current_var->mu_.head(2);
            return pos_diff / globals.T0;
        }
    }
    
    return Eigen::Vector2d::Zero();
}

Eigen::Vector2d Robot::getDesiredVelocity() const {
    // 从规划路径获取期望速度
    if (variables_.size() >= 1) {
        auto current_var = getVar(0);
        if (current_var && current_var->valid_ && current_var->mu_.size() >= 4) {
            return current_var->mu_.tail(2); // 假设mu_是[x, y, vx, vy]格式
        }
    }
    
    // 备选方案：简单计算朝向下一个waypoint的速度
    if (waypoints_.size() > 0) {
        Eigen::Vector2d target = waypoints_[0].head(2);
        Eigen::Vector2d current = position_.head(2);
        Eigen::Vector2d direction = target - current;
        
        if (direction.norm() > 0.1) {
            return direction.normalized() * globals.MAX_SPEED;
        }
    }
    
    return Eigen::Vector2d::Zero();
}

/*******************************************************************************************/
// Function for determining the timesteps at which variables in the planned path are placed.
/*******************************************************************************************/
std::vector<int> Robot::getVariableTimesteps(int lookahead_horizon, int lookahead_multiple){
    // For a lookahead_multiple of 3, variables are spaced at timesteps:
    // Timesteps
    // 0,    1, 2, 3,    5, 7, 9,    12, 15, 18, ...
    // 
    // eg. variables are in groups of size lookahead_multiple.
    // the spacing within a group increases by one each time (1 for the first group, 2 for the second ...)
    // Seems convoluted, but the reasoning was:
    //      the first variable should always be at 1 timestep from the current state (0).
    //      the first few variables should be close together in time
    //      the variables should all be at integer timesteps, but the spacing should sort of increase exponentially.
    std::vector<int> var_list{};
    int N = 1 + int(0.5*(-1 + sqrt(1 + 8*(float)lookahead_horizon/(float)lookahead_multiple)));

    for (int i=0; i<lookahead_multiple*(N+1); i++){
        int section = int(i/lookahead_multiple);
        int f = (i - section*lookahead_multiple + lookahead_multiple/2.*section)*(section+1);
        if (f>=lookahead_horizon){
            var_list.push_back(lookahead_horizon);
            break;
        }
        var_list.push_back(f);
    }

    return var_list;
};