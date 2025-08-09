/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <Robot.h>

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
             Color color,
             b2World* world) : FactorGraph{rid},
             sim_(sim), rid_(rid),
             waypoints_(waypoints),
             robot_radius_(size), color_(color),
             physicsWorld_(world), usePhysics_(world != nullptr), payload_joint_(nullptr) {

    height_3D_ = robot_radius_;     // Height out of plane for 3d visualisation only

    // Robot will always set its horizon state to move towards the next waypoint.
    // Once this waypoint has been reached, it pops it from the waypoints
    Eigen::VectorXd start = position_ = waypoints_[0];
    waypoints_.pop_front();                             
    auto goal = (waypoints_.size()>0) ? waypoints_[0] : start;
    
    // Initialize trajectory with starting position
    trajectory_.push_back(Eigen::Vector2d(start(0), start(1)));

    if (usePhysics_ && physicsWorld_) {
        createPhysicsBody();
    }

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

    const auto& payloads_ = sim->getPayload();

    // Get payload information for rigid body interpolation
    std::shared_ptr<Payload> payload = nullptr;
    if (!payloads_.empty()) {
        payload = payloads_.begin()->second;
    }

    for (int i = 0; i < num_variables_; i++){
        if (payload && globals.FORMATION == "Payload" && globals.T_HORIZON > 0) {
            // Get local contact point coordinates first and validate
            auto [contact_points, contact_normals] = payload->getContactPointsAndNormals();
            
            // Safe contact point index assignment
            int safe_contact_index = 0;
            if (!contact_points.empty()) {
                safe_contact_index = rid_ % contact_points.size();
            }
            
            if (contact_points.empty()) {
                // Fallback to linear interpolation if no contact points
                mu = start + (horizon - start) * (float)(variable_timesteps[i]/(float)variable_timesteps.back());
            } else {
                // Rigid body interpolation for payload formation
                float t = (float)(variable_timesteps[i] * globals.T0 / globals.T_HORIZON);
                
                // Get payload motion parameters
                Eigen::Vector2d P_start = payload->getPosition();
                Eigen::Vector2d P_target = payload->getTarget();
                Eigen::Quaterniond q_start = payload->current_orientation_;
                Eigen::Quaterniond q_target = payload->getTargetRotation();
                
                // Convert quaternions to rotation angles
                double θ_start = atan2(2.0 * (q_start.w() * q_start.z() + q_start.x() * q_start.y()),
                                      1.0 - 2.0 * (q_start.y() * q_start.y() + q_start.z() * q_start.z()));
                double θ_target = atan2(2.0 * (q_target.w() * q_target.z() + q_target.x() * q_target.y()),
                                       1.0 - 2.0 * (q_target.y() * q_target.y() + q_target.z() * q_target.z()));
                
                // Interpolate payload state
                Eigen::Vector2d P_t = P_start + (P_target - P_start) * t;
                double θ_t = θ_start + (θ_target - θ_start) * t;
                
                // Use safe contact point index
                Eigen::Vector2d contact_point = contact_points[safe_contact_index];
                Eigen::Vector2d r_i = contact_point - P_start;
                
                // Rotation matrix at time t
                Eigen::Matrix2d R_t;
                R_t << cos(θ_t), -sin(θ_t),
                       sin(θ_t),  cos(θ_t);
                
                // Robot position maintaining rigid body constraint
                Eigen::Vector2d robot_pos_t = P_t + R_t * r_i;
                
                // Robot velocity from rigid body motion
                double T_total = globals.T_HORIZON;
                Eigen::Vector2d v_payload = (P_target - P_start) / T_total;
                double ω_payload = (θ_target - θ_start) / T_total;
                
                Eigen::Vector2d r_i_rotated = R_t * r_i;
                Eigen::Vector2d robot_vel_t = v_payload + ω_payload * Eigen::Vector2d(-r_i_rotated.y(), r_i_rotated.x());
                
                // Set complete 4-DOF state
                mu << robot_pos_t.x(), robot_pos_t.y(), robot_vel_t.x(), robot_vel_t.y();
            }
        } else {
            // Original linear interpolation for other formations
            if (variable_timesteps.back() > 0) {
                mu = start + (horizon - start) * (float)(variable_timesteps[i]/(float)variable_timesteps.back());
            } else {
                mu = start;
            }
        }
        
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
    // for (int i = 1; i < num_variables_-1; i++)
    // {
    //     std::vector<std::shared_ptr<Variable>> variables{getVar(i)};
    //     auto fac_obs = std::make_shared<ObstacleFactor>(sim, sim->next_fid_++, rid_, variables, globals.SIGMA_FACTOR_OBSTACLE, Eigen::VectorXd::Zero(1), &(sim_->obstacleImg));

    //     // Add this factor to the variable's list of adjacent factors, as well as to the robot's list of factors
    //     for (auto var : fac_obs->variables_) var->add_factor(fac_obs);
    //     this->factors_[fac_obs->key_] = fac_obs;
    // }

    for (int i = 1; i < num_variables_-1; i++) {
        int num_obs = sim_->obstacles_.size();
        std::vector<std::shared_ptr<Variable>> variables{getVar(i)};
        for (int o_id = 0; o_id < num_obs; o_id++) {
            std::shared_ptr<ObsFactor> obs_fac = std::make_shared<ObsFactor>(sim, sim->next_fid_++, rid_, o_id, variables, globals.SIGMA_FACTOR_OBSTACLE, Eigen::VectorXd::Zero(1));
            for (auto var : obs_fac->variables_) var->add_factor(obs_fac);
            this->factors_[obs_fac->key_] = obs_fac;
        }
    }

    // for (auto& [pid, payload]: payloads_) {
    //     createPayloadFactors(payload);
    // }

};

/***************************************************************************************************/
/* Destructor */
/***************************************************************************************************/
Robot::~Robot(){
    detachFromPayload();
}

// Robot.cpp 中的正确实现

void Robot::attachToPayload(std::shared_ptr<Payload> payload, const Eigen::Vector2d& attach_point) {
    if (!physicsBody_ || !payload->physicsBody_) {
        std::cout << "Error: Cannot attach - missing physics bodies" << std::endl;
        return;
    }
    
    if (payload_joint_) {
        detachFromPayload(); // 先断开现有连接
    }
    
    // 创建焊接关节定义
    b2WeldJointDef jointDef;
    jointDef.bodyA = physicsBody_;           // 机器人
    jointDef.bodyB = payload->physicsBody_;  // payload
    
    // 设置连接点（局部坐标）
    jointDef.localAnchorA = physicsBody_->GetLocalPoint(b2Vec2(attach_point.x(), attach_point.y()));
    jointDef.localAnchorB = payload->physicsBody_->GetLocalPoint(b2Vec2(attach_point.x(), attach_point.y()));

    
    
    // 设置相对角度（保持当前相对角度）
    jointDef.referenceAngle = payload->physicsBody_->GetAngle() - physicsBody_->GetAngle();
    
    // 使用正确的参数名
    // jointDef.stiffness = std::numeric_limits<float>::max();  // 刚度 (N*m) - 数值越大越"硬"
    // jointDef.damping = 1000;     // 阻尼 (N*m*s) - 防止震荡
    jointDef.stiffness = 10000;  // 刚度 (N*m) - 数值越大越"硬"
    jointDef.damping = 10000;     // 阻尼 (N*m*s) - 防止震荡
    
    // 创建关节
    payload_joint_ = (b2WeldJoint*)physicsWorld_->CreateJoint(&jointDef);
}

void Robot::detachFromPayload() {
    if (payload_joint_ && physicsWorld_) {
        physicsWorld_->DestroyJoint(payload_joint_);
        payload_joint_ = nullptr;
        std::cout << "Robot " << rid_ << " detached from payload" << std::endl;
    }
}



bool Robot::isConnectedToPayload(int payload_id) const {
    return std::find(connected_payload_ids_.begin(), connected_payload_ids_.end(), payload_id) 
           != connected_payload_ids_.end();
}


void Robot::createPhysicsBody(){

    if (!physicsWorld_) {
        std::cerr << "Error: physicsWorld_ is null." << std::endl;
        return;
    }

    if (position_.size() < 2) {
        std::cerr << "Error: position_.size() is" << position_.size() << ", expected at least 2." << std::endl;
        return;
    }

    // Create a physics body for the robot
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(position_(0), position_(1));
    physicsBody_ = physicsWorld_->CreateBody(&bodyDef);

    // Create a circle collision shape for the robot
    b2CircleShape circleShape;
    circleShape.m_radius = robot_radius_;

    // Create a fixture definition for the circle shape
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &circleShape;
    fixtureDef.density = 0.0f;
    fixtureDef.friction = 0.0f;
    fixtureDef.restitution = 0.0f;

    physicsBody_->CreateFixture(&fixtureDef);
    
    // physicsBody_->SetLinearDamping(0.0f);
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
    
    // Track trajectory for visualization
    trajectory_.push_back(Eigen::Vector2d(position_(0), position_(1)));
    
    // Limit trajectory length to prevent memory issues
    if (trajectory_.size() > 1000) {
        trajectory_.erase(trajectory_.begin());
    }
    
    // // Update physics body state to match logical state
    // print the position_(2) and position_(3) values
    syncLogicalToPhysics();
};

void Robot::syncLogicalToPhysics(){
    if (!usePhysics_ || !physicsBody_) return;
    
    // physicsBody_->SetTransform(b2Vec2(position_(0), position_(1)), 0.0f);
    Eigen::VectorXd increment = ((*this)[1]->mu_ - (*this)[0]->mu_) * globals.TIMESTEP / globals.T0;
    b2Vec2 desiredVel(getVar(0)->mu_(2), getVar(0)->mu_(3));

    physicsBody_->SetLinearVelocity(desiredVel);
}

void Robot::syncPhysicsToLogical(){
    if (!usePhysics_ || !physicsBody_) return;

    b2Vec2 phyPos = physicsBody_->GetPosition();
    position_[0] = phyPos.x;
    position_[1] = phyPos.y;

}

/***************************************************************************************************/
/* Change the prior of the Horizon state */
/***************************************************************************************************/
void Robot::updateHorizon(){
    // Horizon state moves towards the next waypoint.
    // The Horizon state's velocity is capped at MAX_SPEED
    auto horizon = getVar(-1);      // get horizon state variable
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
    // for (auto rid : neighbours_){
    //     if (std::find(connected_r_ids_.begin(), connected_r_ids_.end(), rid)==connected_r_ids_.end()){
    //         createInterrobotFactors(sim_->robots_.at(rid));
    //         if (!sim_->symmetric_factors) sim_->robots_.at(rid)->connected_r_ids_.push_back(rid_);
    //     };
    // }
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
    // Draw planned path
    if (globals.DRAW_PATH){
        static int debug = 0;
        for (auto [vid, variable] : variables_){
            if (!variable->valid_) continue;
            DrawSphere(Vector3{(float)variable->mu_(0), height_3D_, (float)variable->mu_(1)}, 0.5*robot_radius_, ColorAlpha(col, 0.5));
        }
        for (auto [fid, factor] : factors_) factor->draw();
    }     
    // Draw connected robots
    if (globals.DRAW_INTERROBOT){
        for (auto rid : connected_r_ids_){
            if (!interrobot_comms_active_ || !sim_->robots_.at(rid)->interrobot_comms_active_) continue;
            DrawCylinderEx(Vector3{(float)position_(0), height_3D_, (float)position_(1)},
                            Vector3{(float)(*sim_->robots_.at(rid))[0]->mu_(0), sim_->robots_.at(rid)->height_3D_, (float)(*sim_->robots_.at(rid))[0]->mu_(1)}, 
                            0.1, 0.1, 4, BLACK);
        }
    }

    // Draw the waypoints of the robot
    if (globals.DRAW_WAYPOINTS){
        for (int wp_idx=0; wp_idx<waypoints_.size(); wp_idx++){
            DrawCubeV(Vector3{(float)waypoints_[wp_idx](0), height_3D_, (float)waypoints_[wp_idx](1)}, Vector3{1.f*robot_radius_,1.f*robot_radius_,1.f*robot_radius_}, col);
        }
    }
    
    // Draw trajectory history in robot's own color
    if (globals.DRAW_PATH && trajectory_.size() > 1) {
        Color trail_color = ColorAlpha(col, 0.8f);
        
        for (size_t i = 1; i < trajectory_.size(); i++) {
            Vector3 start = {(float)trajectory_[i-1].x(), 0.1f, (float)trajectory_[i-1].y()};
            Vector3 end = {(float)trajectory_[i].x(), 0.1f, (float)trajectory_[i].y()};
            
            DrawCylinderEx(start, end, 0.05f, 0.05f, 4, trail_color);
        }
    }

    // Draw the actual position of the robot. This uses the robotModel defined in Graphics.cpp, others can be used.
    DrawModel(sim_->graphics->robotModel_, Vector3{(float)position_(0), height_3D_, (float)position_(1)}, robot_radius_, col);
};

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