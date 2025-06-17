/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <iostream>
#include <gbp/GBPCore.h>
#include <Simulator.h>
#include <Graphics.h>
#include <Robot.h>
#include <Payload.h>
#include <nanoflann.h>
#include "box2d/box2d.h"


/*******************************************************************************/
// Raylib setup
/*******************************************************************************/
Simulator::Simulator(){
    SetTraceLogLevel(LOG_ERROR);  
    if (globals.DISPLAY){
        SetTargetFPS(60);
        InitWindow(globals.SCREEN_SZ, globals.SCREEN_SZ, globals.WINDOW_TITLE);
    }

    // Initialise kdtree for storing robot positions (needed for nearest neighbour check)
    treeOfRobots_ = new KDTree(2, robot_positions_, 50);  

    // For display only
    // User inputs an obstacle image where the obstacles are BLACK and background is WHITE.
    obstacleImg = LoadImage(globals.OBSTACLE_FILE.c_str());
    if (obstacleImg.width==0) obstacleImg = GenImageColor(globals.WORLD_SZ, globals.WORLD_SZ, WHITE);

    // However for calculation purposes the image needs to be inverted.
    ImageColorInvert(&obstacleImg);
    graphics = new Graphics(obstacleImg);

    if (globals.FORMATION == "Payload"){
        createPayload(Eigen::Vector2d(0., 0.), globals.PAYLOAD_WIDTH, globals.PAYLOAD_HEIGHT);
    }
};



void Simulator::createPayload(Eigen::Vector2d position, float width, float height) {
    auto payload = std::make_shared<Payload>(this, next_payload_id_++, position, width, height, globals.PAYLOAD_DENSITY);
    payloads_[payload->payload_id_] = payload;
    
    // 计算绝对目标位置（基于相对位置）
    Eigen::Vector2d absolute_target_position = position + Eigen::Vector2d(globals.TARGET_RELATIVE_X, globals.TARGET_RELATIVE_Y);
    
    // 设置目标位置
    payload->setTarget(absolute_target_position);
    
    // 设置目标朝向（如果payload支持）
    // 关键修复：使用相对旋转设置目标朝向
    if (std::abs(globals.TARGET_RELATIVE_ROTATION) > 0.001) {  // 如果有旋转目标
        payload->setTargetFromRelativeRotation(globals.TARGET_RELATIVE_ROTATION);
        
        std::cout << "Payload target: position(" << absolute_target_position.x() << ", " << absolute_target_position.y() 
                  << ") relative_rotation(" << globals.TARGET_RELATIVE_ROTATION << " rad)" << std::endl;
    } else {
        // 如果没有旋转目标，目标朝向等于初始朝向
        payload->target_orientation_ = payload->initial_orientation_;
        std::cout << "Payload target: position only, no rotation" << std::endl;
    }
}

/*******************************************************************************/
// Destructor
/*******************************************************************************/
Simulator::~Simulator(){
    delete treeOfRobots_;
    int n = robots_.size();
    for (int i = 0; i < n; ++i) robots_.erase(i);

    if(physicsWorld_) {
        delete physicsWorld_;
        physicsWorld_ = nullptr;
    }

    if (globals.DISPLAY) {
        delete graphics;
        CloseWindow();
    }
};

/*******************************************************************************/
// Drawing graphics.
/*******************************************************************************/
void Simulator::draw(){
    if (!globals.DISPLAY) return;
    
    BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(graphics->camera3d);
            // Draw Ground
            DrawModel(graphics->groundModel_, graphics->groundModelpos_, 1., WHITE);
            
            // Draw Robots
            for (auto [rid, robot] : robots_) robot->draw();
            
            // Draw Payloads
            for (auto [pid, payload]: payloads_) {
                payload->draw();
            }
            
            // Draw Contact points and normals (old method - real robot-payload contacts)
            for (auto [rid, robot] : robots_) {
                for (auto [pid, payload] : payloads_) {
                    if (isRobotContactingPayload(rid, pid)) {
                        auto [contactPoint, contactNormal] = getContactPoint(rid, pid);
                        if (contactPoint == Eigen::Vector2d::Zero()) continue;
                        
                        // 绘制真实接触点
                        Vector3 contactPose3D = {
                            (float) contactPoint.x(),
                            1.5f,
                            (float) contactPoint.y()
                        };
                        DrawSphere(contactPose3D, 0.5f, RED);
                        
                        // 绘制真实接触法向量
                        Vector3 normalEnd = {
                            (float) (contactPoint.x() + contactNormal.x()),
                            1.5f,
                            (float) (contactPoint.y() + contactNormal.y())
                        };
                        DrawLine3D(contactPose3D, normalEnd, BLUE);
                    }
                }
            }
            
            // Draw fixed contact points and normals from Payload::getContactPointsAndNormals()
            for (auto [pid, payload] : payloads_) {
                auto [fixedContactPoints, fixedContactNormals] = payload->getContactPointsAndNormals();
                
                for (size_t i = 0; i < fixedContactPoints.size(); i++) {
                    // 绘制固定接触点
                    Vector3 fixedPointPos3D = {
                        (float) fixedContactPoints[i].x(),
                        1.8f, // 稍微高一点以区分
                        (float) fixedContactPoints[i].y()
                    };
                    DrawSphere(fixedPointPos3D, 0.4f, YELLOW); // 用黄色区分
                    
                    // 绘制固定接触点的法向量
                    Vector3 fixedNormalEnd = {
                        (float) (fixedContactPoints[i].x() + fixedContactNormals[i].x()),
                        1.8f,
                        (float) (fixedContactPoints[i].y() + fixedContactNormals[i].y())
                    };
                    DrawLine3D(fixedPointPos3D, fixedNormalEnd, GREEN); // 用绿色表示固定法向量
                }
            }
            
        EndMode3D();
        draw_info(clock_);
    EndDrawing();
};

/*******************************************************************************/
// Timestep loop of simulator.
/*******************************************************************************/
void Simulator::moveRobot(){
    // Update the robot current and horizon states by one timestep

    for (int i=0; i<globals.NUM_ITERS; i++){
        iterateGBP(1, INTERNAL, robots_);
        // iterateGBP(1, EXTERNAL, robots_);
    }
    for (auto [r_id, robot] : robots_) {
        robot->updateHorizon();
        robot->updateCurrent();
    }

    // Increase simulation clock by one timestep
    clock_++;
    // print the clock_
    if (clock_ >= globals.MAX_TIME ) globals.RUN = false;
}

bool Simulator::isRobotContactingPayload(int robot_id, int payload_id) {
    // 获取机器人和payload的物理体
    auto robot_it = robots_.find(robot_id);
    auto payload_it = payloads_.find(payload_id);
    
    if (robot_it == robots_.end() || payload_it == payloads_.end()) {
        return false; // 机器人或payload不存在
    }
    
    b2Body* robot_body = robot_it->second->physicsBody_;
    b2Body* payload_body = payload_it->second->physicsBody_;
    
    if (!robot_body || !payload_body) {
        return false; // 物理体不存在
    }
    
    // 遍历机器人的所有接触
    for (b2ContactEdge* edge = robot_body->GetContactList(); edge; edge = edge->next) {
        if (edge->other == payload_body && edge->contact->IsTouching()) {
            return true; // 找到接触
        }
    }
    
    return false; // 没有接触
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> Simulator::getContactPoint(int robot_id, int payload_id){
    auto robot_it = robots_.find(robot_id);
    auto payload_it = payloads_.find(payload_id);

    b2Body* robot_body = robot_it->second->physicsBody_;
    b2Body* payload_body = payload_it->second->physicsBody_;

    for (b2ContactEdge *edge = robot_body->GetContactList(); edge; edge=edge->next){
        b2WorldManifold worldManifold;
        edge->contact->GetWorldManifold(&worldManifold);

        b2Vec2 contactPoint = worldManifold.points[0];
        Eigen::Vector2d point(contactPoint.x, contactPoint.y);
        b2Vec2 normal = worldManifold.normal;
        if (edge->contact->GetFixtureA()->GetBody() == robot_body) {
            return {point, Eigen::Vector2d(normal.x, normal.y)};
        } else{
            return {point, Eigen::Vector2d(-normal.x, -normal.y)};
        }
    }
    return {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
}

Eigen::VectorXd Simulator::solveConstrainedLeastSquares(const Eigen::MatrixXd &G, const Eigen::Vector3d& w_cmd) {
    // 检查是否有接触的机器人
    if (G.cols() == 0) {
        return Eigen::VectorXd::Zero(0);
    }
    
    // 使用SVD求解（最稳定）
    Eigen::VectorXd f = G.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(w_cmd);
    
    // 简单处理非负约束
    for (int i = 0; i < f.size(); i++) {
        f(i) = std::max(0.0, f(i));
    }
    
    // 调试输出
    std::cout << "G matrix (3x" << G.cols() << "):\n" << G << std::endl;
    std::cout << "w_cmd: " << w_cmd.transpose() << std::endl;
    std::cout << "Solved f: " << f.transpose() << std::endl;
    std::cout << "Residual: " << (G * f - w_cmd).norm() << std::endl;
    
    return f;
}

void Simulator::computeLeastSquares(){
    for (auto& [pid, payload]: payloads_){
        // 计算期望速度和角速度
        Eigen::Vector2d dist_to_target = payload->target_position_ - payload->position_;
        Eigen::Vector2d desired_velocity = dist_to_target.normalized() * 
            std::min(static_cast<double>(globals.MAX_SPEED), dist_to_target.norm());

        Eigen::Quaterniond relative_rotation = payload->target_orientation_ * payload->current_orientation_.inverse();

        double rotation_error = 2.0 * std::acos(std::abs(relative_rotation.w()));
        double rotation_direction = (relative_rotation.z() >= 0) ? 1.0 : -1.0;
        double signed_rotation_error = rotation_direction * rotation_error;

        double desired_angular_velocity = std::copysign(1.0, signed_rotation_error) * 
            std::min(static_cast<double>(globals.MAX_ANGULAR_SPEED), std::abs(signed_rotation_error));
        
        // 获取当前状态
        Eigen::Vector2d current_velocity = payload->getVelocity();
        double current_angular_velocity = payload->getAngularVelocity();

        // 计算所需的力和力矩
        double dt = globals.TIMESTEP;
        double mass = payload->getMass();
        double inertia = payload->getMomentOfInertia();

        Eigen::Vector2d required_force = (mass * (desired_velocity - current_velocity)) / dt;
        double required_torque = (inertia * (desired_angular_velocity - current_angular_velocity)) / dt;
        
        // 使用固定的接触点和法向量
        auto [contact_points, contact_normals] = payload->getContactPointsAndNormals();
        
        // 构建G矩阵
        int num_contact_points = contact_points.size();
        if (num_contact_points == 0) continue; // 没有接触点，跳过
        
        Eigen::MatrixXd G(3, num_contact_points);
        Eigen::Vector2d payload_center = payload->getPosition();
        
        for (int i = 0; i < num_contact_points; i++){
            G(0, i) = contact_normals[i].x();
            G(1, i) = contact_normals[i].y();

            Eigen::Vector2d r = contact_points[i] - payload_center;
            double torque_contribution = r.x() * contact_normals[i].y() - r.y() * contact_normals[i].x();
            G(2, i) = torque_contribution;
        }

        // 构建命令向量
        Eigen::Vector3d w_cmd;
        w_cmd << required_force.x(), required_force.y(), required_torque;
        
        // 求解最小二乘问题
        Eigen::VectorXd f = solveConstrainedLeastSquares(G, w_cmd);
        
        // 直接将计算出的力施加到payload上
        applyForcesToPayload(payload, f, contact_points, contact_normals);
        std::cout << "Solved forces: " << f.transpose() << std::endl;
    }
}

void Simulator::applyForcesToPayload(std::shared_ptr<Payload> payload, 
                                   const Eigen::VectorXd& forces,
                                   const std::vector<Eigen::Vector2d>& contact_points,
                                   const std::vector<Eigen::Vector2d>& contact_normals) {
    if (!payload->physicsBody_) return;
    
    // 遍历所有接触点，施加对应的力
    for (int i = 0; i < forces.size() && i < contact_points.size(); i++) {
        if (forces(i) > 0.01) { // 只施加显著的力
            // 计算力向量
            Eigen::Vector2d force_vector = forces(i) * contact_normals[i];
            
            // 转换为Box2D格式
            b2Vec2 force_b2(force_vector.x(), force_vector.y());
            b2Vec2 point_b2(contact_points[i].x(), contact_points[i].y());
            
            // 施加力到指定点
            payload->physicsBody_->ApplyForce(force_b2, point_b2, true);
        }
    }
}


void Simulator::applyDirectPayloadVelocityControl() {
    for (auto& [pid, payload] : payloads_) {
        if (!payload->physicsBody_) continue;
        
        // 计算期望的线速度和角速度
        Eigen::Vector2d desired_velocity = computeDesiredPayloadVelocity(payload);
        double desired_angular_velocity = computeDesiredPayloadAngularVelocity(payload);
        
        // 直接设置payload的速度（Box2D提供的功能）
        b2Vec2 new_linear_velocity(desired_velocity.x(), desired_velocity.y());
        payload->physicsBody_->SetLinearVelocity(new_linear_velocity);
        payload->physicsBody_->SetAngularVelocity(desired_angular_velocity);
        
        // 调试输出
        static int debug_counter = 0;
        if (debug_counter++ % 60 == 0) {  // 每60帧输出一次
            b2Vec2 current_velocity = payload->physicsBody_->GetLinearVelocity();
            double current_angular_velocity = payload->physicsBody_->GetAngularVelocity();
            
            std::cout << "Direct velocity control - Set vel: (" << desired_velocity.x() << ", " << desired_velocity.y() 
                      << "), Actual vel: (" << current_velocity.x << ", " << current_velocity.y << ")" << std::endl;
            std::cout << "Angular - Set: " << desired_angular_velocity 
                      << ", Actual: " << current_angular_velocity << std::endl;
        }
    }
}

Eigen::Vector2d Simulator::computeDesiredPayloadVelocity(std::shared_ptr<Payload> payload) {
    // 与PayloadVelocityFactor::computeDesiredPayloadMotion()保持一致的线速度计算
    Eigen::Vector2d payload_to_target = payload->target_position_ - payload->position_;
    
    if (payload_to_target.norm() <= 0.1) {
        return Eigen::Vector2d::Zero();  // 与factor中的阈值保持一致
    }
    
    // 完全复制factor中的逻辑
    Eigen::Vector2d desired_velocity = payload_to_target.normalized() * globals.MAX_SPEED * 0.5;
    
    return desired_velocity;
}

double Simulator::computeDesiredPayloadAngularVelocity(std::shared_ptr<Payload> payload) {
    // 与PayloadVelocityFactor::computeDesiredPayloadMotion()保持一致的角速度计算
    double rotation_error = payload->getRotationError();
    
    if (std::abs(rotation_error) <= 0.01) {  // 与factor中的阈值保持一致
        return 0.0;
    }
    
    // 完全复制factor中的逻辑
    double desired_angular_velocity = std::copysign(1.0, rotation_error) * 
        std::min(static_cast<double>(globals.MAX_ANGULAR_SPEED * 0.5), std::abs(rotation_error));
    
    // 添加相同的调试输出
    static int debug_counter = 0;
    if (debug_counter++ % 60 == 0) {
        std::cout << "Direct control - Rotation error: " << rotation_error 
                  << " rad, desired angular vel: " << desired_angular_velocity << std::endl;
    }
    
    return desired_angular_velocity;
}


// 修改timestep方法
void Simulator::timestep() {
    if (globals.SIM_MODE != Timestep) return;
    
    // 根据配置选择控制方法
    if (globals.USE_DIRECT_PAYLOAD_VELOCITY) {
        // 直接速度控制模式
        applyDirectPayloadVelocityControl();
        std::cout << "Using direct payload velocity control" << std::endl;
    } else if (globals.USE_DISTRIBUTED_PAYLOAD_CONTROL) {
        // 分布式GBP控制模式
        updateDistributedPayloadControl();
        std::cout << "Using distributed GBP control" << std::endl;
    } else {
        // 原有的集中式最小二乘控制
        computeLeastSquares();
        std::cout << "Using centralized least squares control" << std::endl;
    }
    
    calculateRobotNeighbours(robots_);
    
    // 更新因子（只在GBP模式下需要）
    if (globals.USE_DISTRIBUTED_PAYLOAD_CONTROL && !globals.USE_DIRECT_PAYLOAD_VELOCITY) {
        for (auto [r_id, robot] : robots_) {
            robot->updatePayloadFactors(payloads_);
            robot->updateInterrobotFactors();
        }
    }
    
    setCommsFailure(globals.COMMS_FAILURE_RATE);
    
    // GBP迭代（只在GBP模式下需要）
    if (globals.USE_DISTRIBUTED_PAYLOAD_CONTROL && !globals.USE_DIRECT_PAYLOAD_VELOCITY) {
        for (int i = 0; i < globals.NUM_ITERS; i++) {
            iterateGBP(1, INTERNAL, robots_);
            iterateGBP(1, EXTERNAL, robots_);
        }
        
        // 更新机器人状态
        for (auto [r_id, robot] : robots_) {
            robot->updateHorizon();
            robot->updateCurrent();
        }
    }
    
    // 物理世界更新
    if (physicsWorld_) {
        physicsWorld_->Step(globals.TIMESTEP, 8, 3);
        for (auto [r_id, robot] : robots_) {
            robot->syncPhysicsToLogical();
        }
        for (auto [p_id, payload] : payloads_) {
            payload->update();
        }
    }
    
    clock_++;
    if (clock_ >= globals.MAX_TIME) globals.RUN = false;
}

// 简化 updateDistributedPayloadControl()
void Simulator::updateDistributedPayloadControl() {
    // 简单的状态监控
    for (auto& [pid, payload] : payloads_) {
        int connected_robots = 0;
        for (auto& [rid, robot] : robots_) {
            if (robot->isConnectedToPayload(pid)) {
                connected_robots++;
            }
        }
        
        // if (connected_robots > 0) {
        //     std::cout << "Payload " << pid << " has " << connected_robots 
        //               << " robots connected via factors" << std::endl;
        // }
    }
}

/*******************************************************************************/
// Use a kd-tree to perform a radius search for neighbours of a robot within comms. range
// (Updates the neighbours_ of a robot)
/*******************************************************************************/
void Simulator::calculateRobotNeighbours(std::map<int,std::shared_ptr<Robot>>& robots){
    for (auto [rid, robot] : robots){
        robot_positions_.at(rid) = std::vector<double>{robot->position_(0), robot->position_(1)};
    }
    treeOfRobots_->index->buildIndex(); 

    for (auto [rid, robot] : robots){
        // Find nearest neighbors in radius
        robot->neighbours_.clear();
        std::vector<double> query_pt = std::vector<double>{robots[rid]->position_(0), robots[rid]->position_(1)};
        const float search_radius = pow(globals.COMMUNICATION_RADIUS,2.);
        std::vector<nanoflann::ResultItem<size_t, double>> matches;
        nanoflann::SearchParameters params; params.sorted = true;
        const size_t nMatches = treeOfRobots_->index->radiusSearch(&query_pt[0], search_radius, matches, params);
        for(size_t i = 0; i < nMatches; i++){
            auto it = robots_.begin(); std::advance(it, matches[i].first);
            if (it->first==rid) continue;
            robot->neighbours_.push_back(it->first);
        }
    }
};

/*******************************************************************************/
// Set a proportion of robots to not perform inter-robot communications
/*******************************************************************************/
void Simulator::setCommsFailure(float failure_rate){
    if (failure_rate==0) return;
    // Get all the robot ids and then shuffle them      
    std::vector<int> range{}; for (auto& [rid, robot] : robots_) range.push_back(rid);
    std::shuffle(range.begin(), range.end(), gen_uniform);
    // Set a proportion of the robots as inactive using their interrobot_comms_active_ flag.
    int num_inactive = round(failure_rate*robots_.size());
    for (int i=0; i<range.size(); i++){
        robots_.at(range[i])->interrobot_comms_active_ = (i>=num_inactive);
    }
}

/*******************************************************************************/
// Handles keypresses and mouse input, and updates camera.
/*******************************************************************************/
void Simulator::eventHandler(){
    // Deal with Keyboard key press
    int key = GetKeyPressed();
    switch (key)
    {
    case KEY_ESCAPE:
            globals.RUN = false;                                                    break;
    case KEY_H:
            globals.LAST_SIM_MODE = (globals.SIM_MODE==Help) ? globals.LAST_SIM_MODE : globals.SIM_MODE;
            globals.SIM_MODE = (globals.SIM_MODE==Help) ? globals.LAST_SIM_MODE: Help;break;
    case KEY_SPACE:
            graphics->camera_transition_ = !graphics->camera_transition_;           break;
    case KEY_P:
            globals.DRAW_PATH = !globals.DRAW_PATH;                                 break;
    case KEY_R:
            globals.DRAW_INTERROBOT = !globals.DRAW_INTERROBOT;                                   break;
    case KEY_W:
            globals.DRAW_WAYPOINTS = !globals.DRAW_WAYPOINTS;                                 break;
    case KEY_ENTER:
            globals.SIM_MODE  = (globals.SIM_MODE==Timestep) ? SimNone : Timestep;  break;
    default:
        break;
    }

    // Mouse input handling
    Ray ray = GetMouseRay(GetMousePosition(), graphics->camera3d);
    Vector3 mouse_gnd = Vector3Add(ray.position, Vector3Scale(ray.direction, -ray.position.y/ray.direction.y));
    Vector2 mouse_pos{mouse_gnd.x, mouse_gnd.z};        // Position on the ground plane
    // Do stuff with mouse here using mouse_pos .eg:
    // if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)){
    //     do_code
    // }

    // Update the graphics if the camera has moved
    graphics->update_camera();
}

void Simulator::createSingleRobot(){
    // Create a single robot at the specified position
    if (!new_robots_needed_) return;
    new_robots_needed_ = false;
    std::vector<std::shared_ptr<Robot>> robots_to_create{};
    Eigen::VectorXd starting = Eigen::VectorXd{{-25., 0., 0.,0.}};
    Eigen::VectorXd ending = Eigen::VectorXd{{25., 0., 0.,0.}};
    // print the starting and ending positions
    std::cout << "Starting: " << starting.transpose() << std::endl;
    std::cout << "Ending: " << ending.transpose() << std::endl;
    std::deque<Eigen::VectorXd> waypoints{starting, ending};
    float robot_radius = globals.ROBOT_RADIUS;
    Color robot_color = ColorFromHSV(0, 1., 0.75);
    robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints, robot_radius, robot_color, getPhysicsWorld()));
    for (auto& robot : robots_to_create) {
        robots_[robot->rid_] = robot;
        robot_positions_[robot->rid_] = std::vector<double>{robot->position_(0), robot->position_(1)};
    }
}

/*******************************************************************************/
// Create new robots if needed. Handles deletion of robots out of bounds. 
// New formations must modify the vectors "robots to create" and optionally "robots_to_delete"
// by appending (push_back()) a shared pointer to a Robot class.
/*******************************************************************************/


void Simulator::createOrDeleteRobots(){
    if (!new_robots_needed_) return;

    std::vector<std::shared_ptr<Robot>> robots_to_create{};
    std::vector<std::shared_ptr<Robot>> robots_to_delete{};
    Eigen::VectorXd starting, turning, ending; // Waypoints : [x,y,xdot,ydot].

    if (globals.FORMATION=="circle"){
    // Robots must travel to opposite sides of circle
        new_robots_needed_ = false;
        float min_circumference_spacing = 5.*globals.ROBOT_RADIUS;
        double min_radius = 0.25 * globals.WORLD_SZ;
        Eigen::VectorXd centre{{0., 0., 0.,0.}};
        for (int i=0; i<globals.NUM_ROBOTS; i++){
            // Select radius of large circle to be at least min_radius,
            // Also ensures that robots in the circle are at least min_circumference_spacing away from each other
            float radius_circle = (globals.NUM_ROBOTS==1) ? min_radius : 
                std::max(min_radius, sqrt(min_circumference_spacing / (2. - 2.*cos(2.*PI/(double)globals.NUM_ROBOTS))));
            Eigen::VectorXd offset_from_centre = Eigen::VectorXd{{radius_circle * cos(2.*PI*i/(float)globals.NUM_ROBOTS)},
                                                            {radius_circle * sin(2.*PI*i/(float)globals.NUM_ROBOTS)},
                                                            {0.},{0.}};
            starting = centre + offset_from_centre;
            ending = centre - offset_from_centre;
            // print the starting and ending positions
            std::cout << "Starting: " << starting.transpose() << std::endl;
            std::cout << "Ending: " << ending.transpose() << std::endl;
            std::deque<Eigen::VectorXd> waypoints{starting, ending};
            
            // Define robot radius and colour here.
            float robot_radius = globals.ROBOT_RADIUS;
            Color robot_color = ColorFromHSV(i*360./(float)globals.NUM_ROBOTS, 1., 0.75);
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints, robot_radius, robot_color, getPhysicsWorld()));
        }
    } else if (globals.FORMATION == "Payload") {
        new_robots_needed_ = false;
        float robot_radius = globals.ROBOT_RADIUS;
        
        // 获取payload信息
        if (payloads_.empty()) return;
        
        auto payload = payloads_.begin()->second;
        auto [contact_points, contact_normals] = payload->getContactPointsAndNormals();
        
        if (contact_points.empty()) {
            std::cout << "Warning: No contact points available for payload" << std::endl;
            return;
        }
        
        // 为每个机器人分配一个接触点索引
        for (int i = 0; i < globals.NUM_ROBOTS; i++) {
            int contact_point_index = i % contact_points.size();
            
            // 获取对应的接触点和法向量
            Eigen::Vector2d contact_point = contact_points[contact_point_index];
            Eigen::Vector2d contact_normal = contact_normals[contact_point_index];
            
            // 机器人初始位置：接触点 - 机器人半径 * 法向量
            Eigen::Vector2d robot_start_pos = contact_point - robot_radius * contact_normal;
            
            Eigen::VectorXd starting(4);
            starting << robot_start_pos.x(), robot_start_pos.y(), 0.0, 0.0;
            
            Eigen::VectorXd ending = starting;
            
            std::deque<Eigen::VectorXd> waypoints{starting, ending};
            
            Color robot_color = ColorFromHSV(contact_point_index * 360.0f / contact_points.size(), 1.0f, 0.75f);
            
            auto robot = std::make_shared<Robot>(this, next_rid_++, waypoints, robot_radius, robot_color, getPhysicsWorld());
            
            // 分配接触点索引
            robot->assigned_contact_point_index_ = contact_point_index;
            
            robots_to_create.push_back(robot);
            
            std::cout << "Robot " << robot->rid_ << " assigned to contact point " << contact_point_index 
                      << " at position (" << contact_point.x() << ", " << contact_point.y() << ")" << std::endl;
        }
        
    } else if (globals.FORMATION=="junction"){
    // Robots in a cross-roads style junction. There is only one-way traffic, and no turning.        
        new_robots_needed_ = true;      // This is needed so that more robots can be created as the simulation progresses.
        if (clock_%20==0){              // Arbitrary condition on the simulation time to create new robots
            int n_roads = 2;
            int road = random_int(0,n_roads-1);
            Eigen::Matrix4d rot; rot.setZero();
            rot.topLeftCorner(2,2) << cos(PI/2.*road), -sin(PI/2.*road), sin(PI/2.*road), cos(PI/2.*road);
            rot.bottomRightCorner(2,2) << cos(PI/2.*road), -sin(PI/2.*road), sin(PI/2.*road), cos(PI/2.*road);

            int n_lanes = 2;
            int lane = random_int(0,n_lanes-1);
            double lane_width = 4.*globals.ROBOT_RADIUS;
            double lane_v_offset = (0.5*(1-n_lanes)+lane)*lane_width;
            starting = rot * Eigen::VectorXd{{-globals.WORLD_SZ/2., lane_v_offset, globals.MAX_SPEED, 0.}};
            ending = rot * Eigen::VectorXd{{(double)globals.WORLD_SZ, lane_v_offset, 0., 0.}};
            std::deque<Eigen::VectorXd> waypoints{starting, ending};
            float robot_radius = globals.ROBOT_RADIUS;
            Color robot_color = DARKGREEN;
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints, robot_radius, robot_color, getPhysicsWorld()));
        }

        // Delete robots if out of bounds
        for (auto [rid, robot] : robots_){
            if (abs(robot->position_(0))>globals.WORLD_SZ/2 || abs(robot->position_(1))>globals.WORLD_SZ/2){
                robots_to_delete.push_back(robot);
            }
        }

    } else if (globals.FORMATION=="junction_twoway"){
    // Robots in a two-way junction, turning LEFT (RED), RIGHT (BLUE) or STRAIGHT (GREEN)
        new_robots_needed_ = true;   // This is needed so that more robots can be created as the simulation progresses.
        if (clock_%20==0){           // Arbitrary condition on the simulation time to create new robots
            int n_roads = 4;
            int road = random_int(0,n_roads-1);
            // We will define one road (the one going left) and then we can rotate the positions for other roads.
            Eigen::Matrix4d rot; rot.setZero();
            rot.topLeftCorner(2,2) << cos(PI/2.*road), -sin(PI/2.*road), sin(PI/2.*road), cos(PI/2.*road);
            rot.bottomRightCorner(2,2) << cos(PI/2.*road), -sin(PI/2.*road), sin(PI/2.*road), cos(PI/2.*road);

            int n_lanes = 2;
            int lane = random_int(0,n_lanes-1);
            int turn = random_int(0,2);
            double lane_width = 4.*globals.ROBOT_RADIUS;
            double lane_v_offset = (0.5*(1-2.*n_lanes)+lane)*lane_width;
            double lane_h_offset = (1-turn)*(0.5+lane-n_lanes)*lane_width;
            starting = rot * Eigen::VectorXd{{-globals.WORLD_SZ/2., lane_v_offset, globals.MAX_SPEED, 0.}};
            turning = rot * Eigen::VectorXd{{lane_h_offset, lane_v_offset, (turn%2)*globals.MAX_SPEED, (turn-1)*globals.MAX_SPEED}};
            ending = rot * Eigen::VectorXd{{lane_h_offset + (turn%2)*globals.WORLD_SZ*1., lane_v_offset + (turn-1)*globals.WORLD_SZ*1., 0., 0.}};
            std::deque<Eigen::VectorXd> waypoints{starting, turning, ending};
            float robot_radius = globals.ROBOT_RADIUS;
            Color robot_color = ColorFromHSV(turn*120., 1., 0.75);
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints, robot_radius, robot_color, getPhysicsWorld()));
        }
        
        // Delete robots if out of bounds
        for (auto [rid, robot] : robots_){
            if (abs(robot->position_(0))>globals.WORLD_SZ/2 || abs(robot->position_(1))>globals.WORLD_SZ/2){
                robots_to_delete.push_back(robot);
            }
        }
    } else {
        print("Shouldn't reach here, formation not defined!");
        // Define new formations here!
    }        
    
    // Create robots first
    for (auto robot : robots_to_create){
        robot_positions_[robot->rid_] = std::vector<double>{robot->waypoints_[0](0), robot->waypoints_[0](1)};
        robots_[robot->rid_] = robot;
    }
    
    // 关键修改：可切换的刚性连接
    if (globals.FORMATION == "Payload" && !payloads_.empty()) {
        auto payload = payloads_.begin()->second;
        auto [contact_points, contact_normals] = payload->getContactPointsAndNormals();
        
        if (globals.USE_RIGID_ATTACHMENT) {
            // 启用刚性连接
            for (auto robot : robots_to_create) {
                int contact_index = robot->assigned_contact_point_index_;
                if (contact_index < contact_points.size()) {
                    Eigen::Vector2d attach_point = contact_points[contact_index];
                    robot->attachToPayload(payload, attach_point);
                    std::cout << "Robot " << robot->rid_ << " attached to payload via Box2D joint (rigid attachment enabled)" << std::endl;
                }
            }
        } else {
            // 禁用刚性连接，使用纯因子图控制
            std::cout << "Rigid attachment disabled - robots will rely purely on factor-based control" << std::endl;
            for (auto robot : robots_to_create) {
                std::cout << "Robot " << robot->rid_ << " will use factor-based control only" << std::endl;
            }
        }
    }
    
    // Delete robots
    for (auto robot : robots_to_delete){
        deleteRobot(robot);
    }
}

/*******************************************************************************/
// Deletes the robot from the simulator's robots_, as well as any variable/factors associated.
/*******************************************************************************/
void Simulator::deleteRobot(std::shared_ptr<Robot> robot){
    auto connected_rids_copy = robot->connected_r_ids_;
    for (auto r : connected_rids_copy){
        robot->deleteInterrobotFactors(robots_.at(r));
        robots_.at(r)->deleteInterrobotFactors(robot);
    }
    robots_.erase(robot->rid_);
    robot_positions_.erase(robot->rid_);
}

void Simulator::deletePayload(int payload_id) {
    payloads_.erase(payload_id);
}

std::shared_ptr<Payload> Simulator::getPayload(int payload_id) {
    auto it = payloads_.find(payload_id);
    return (it != payloads_.end()) ? it->second : nullptr;
}