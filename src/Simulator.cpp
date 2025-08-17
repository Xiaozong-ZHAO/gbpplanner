/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <iostream>
#include <fstream>
#include <chrono>
#include <gbp/GBPCore.h>
#include <Simulator.h>
#include <Graphics.h>
#include <Robot.h>
#include <RobotGTSAM.h>
#include <Payload.h>
#include <nanoflann.h>
#include "box2d/box2d.h"
#include "json.hpp"

extern Globals globals;

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

    Eigen::Vector2d starting_position = Eigen::Vector2d(globals.STARTING_X, globals.STARTING_Y);
    createPayload(starting_position, globals.PAYLOAD_WIDTH, globals.PAYLOAD_HEIGHT);
    
    // Load obstacles from JSON file
    loadObstacles();
    
    // Initialize CSV export if enabled
    if (globals.EXPORT_TRAJECTORY_DATA) {
        initCSVExport();
    }
};

std::map<int, std::shared_ptr<Payload>> Simulator::getPayload(){
    return payloads_;
}

void Simulator::createPayload(Eigen::Vector2d position, float width, float height) {
    auto payload = std::make_shared<Payload>(this, next_payload_id_++, position, width, height, globals.PAYLOAD_DENSITY);
    payloads_[payload->payload_id_] = payload;
    
    // 计算绝对目标位置（基于相对位置）
    Eigen::Vector2d target_position = Eigen::Vector2d(globals.TARGET_RELATIVE_X, globals.TARGET_RELATIVE_Y);
    
    // 设置目标位置
    payload->setTarget(target_position);

    // 设置目标朝向（如果payload支持）
    // 关键修复：使用相对旋转设置目标朝向
    if (std::abs(globals.TARGET_RELATIVE_ROTATION) > 0.001) {  // 如果有旋转目标
        payload->setTargetFromRelativeRotation(globals.TARGET_RELATIVE_ROTATION);
    } else {
        // 如果没有旋转目标，目标朝向等于初始朝向
        payload->target_orientation_ = payload->initial_orientation_;
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

    // Close CSV file if it was opened
    if (trajectory_csv_file_.is_open()) {
        trajectory_csv_file_.close();
        std::cout << "CSV export completed and file closed" << std::endl;
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
            
            // Draw GTSAM Robots  
            for (auto [rid, robot_gtsam] : robots_gtsam_) robot_gtsam->draw();
            
            // Draw Payloads
            for (auto [pid, payload]: payloads_) {
                payload->draw();
            }
            
            // Draw Obstacles
            drawObstacles();

        EndMode3D();
        draw_info(clock_);
    EndDrawing();
};

/*******************************************************************************/
// Timestep loop of simulator.
/*******************************************************************************/
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


Eigen::MatrixXd Simulator::Quat2Rot(Eigen::Quaterniond q) {
    Eigen::MatrixXd R(2, 2);
    R(0, 0) = q.w() * q.w() + q.x() *q.x() - q.y() * q.y() - q.z() * q.z();
    R(0, 1) = 2 * (q.x() * q.y() - q.w() * q.z());
    R(1, 0) = 2 * (q.x() * q.y() + q.w() * q.z());
    R(1, 1) = q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z();
    return R;
}







// Distributed GBP control only
void Simulator::timestep() {
    if (globals.SIM_MODE != Timestep) return;
    
    // Use distributed GBP control
    updateDistributedPayloadControl();
    
    calculateRobotNeighbours(robots_);
    
    // Update interrobot factors
    for (auto [r_id, robot] : robots_) {
        robot->updateInterrobotFactors();
    }
    
    setCommsFailure(globals.COMMS_FAILURE_RATE);
    
    // Start timing GBP iterations
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // GBP iterations
    for (int i = 0; i < globals.NUM_ITERS; i++) {
        iterateGBP(1, INTERNAL, robots_);
        iterateGBP(1, EXTERNAL, robots_);
    }
    
    // End timing GBP iterations
    auto end_time = std::chrono::high_resolution_clock::now();
    gbp_duration_microseconds_ = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    
    // Update robot states
    for (auto [r_id, robot] : robots_) {
        robot->updateHorizon();
        robot->updateCurrent();
    }
    
    // Step physics world if it exists
    if (physicsWorld_) {
        physicsWorld_->Step(globals.TIMESTEP, 6, 2); // timeStep, velocityIterations, positionIterations
    }
    
    // Sync physics back to logical state for all robots
    for (auto [r_id, robot] : robots_) {
        robot->syncPhysicsToLogical();
    }
    
    // Update payload states from physics
    for (auto& [pid, payload] : payloads_) {
        payload->update();
    }
    
    // Export payload trajectory data to CSV
    exportPayloadTrajectory();
    
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
        // std::cout << "Payload " << pid << " has " << connected_robots << " connected robots." << std::endl;
    }
}

/*******************************************************************************/
// Use a kd-tree to perform a radius search for neighbours of a robot within comms. range
// (Updates the neighbours_ of a robot)
/*******************************************************************************/
  void Simulator::locateNearbys(std::map<int, std::shared_ptr<Robot>>& robots)
   {
      // Clear all nearby lists first
      for (auto [rid, robot] : robots) {
          robot->nearby_.clear();
      }

      // Get all robot positions with their IDs
      std::vector<std::pair<int, Eigen::Vector2d>> robot_positions;
      for (auto [rid, robot] : robots) {
          robot_positions.push_back({rid,
  Eigen::Vector2d(robot->position_(0), robot->position_(1))});
      }

      // Sort robots to identify corners of the square
      // First sort by y-coordinate, then by x-coordinate
      std::sort(robot_positions.begin(), robot_positions.end(),
                [](const std::pair<int, Eigen::Vector2d>& a, const
  std::pair<int, Eigen::Vector2d>& b) {
                    if (std::abs(a.second.y() - b.second.y()) < 1e-6) {
                        return a.second.x() < b.second.x();
                    }
                    return a.second.y() < b.second.y();
                });
      int bottom_left_rid = robot_positions[0].first;
      int bottom_right_rid = robot_positions[1].first;
      int top_left_rid = robot_positions[2].first;
      int top_right_rid = robot_positions[3].first;

      // Set adjacent neighbors for each robot (excluding diagonal)
      // Bottom-left: adjacent to bottom-right and top-left
      robots[bottom_left_rid]->nearby_.push_back(bottom_right_rid);
      robots[bottom_left_rid]->nearby_.push_back(top_left_rid);

      // Bottom-right: adjacent to bottom-left and top-right
      robots[bottom_right_rid]->nearby_.push_back(bottom_left_rid);
      robots[bottom_right_rid]->nearby_.push_back(top_right_rid);

      // Top-left: adjacent to bottom-left and top-right
      robots[top_left_rid]->nearby_.push_back(bottom_left_rid);
      robots[top_left_rid]->nearby_.push_back(top_right_rid);

      // Top-right: adjacent to bottom-right and top-left
      robots[top_right_rid]->nearby_.push_back(bottom_right_rid);
      robots[top_right_rid]->nearby_.push_back(top_left_rid);
  }

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
/*******************************************************************************/
// Create new robots if needed. Handles deletion of robots out of bounds. 
// New formations must modify the vectors "robots to create" and optionally "robots_to_delete"
// by appending (push_back()) a shared pointer to a Robot class.
/*******************************************************************************/


// MAIN BRANCH MARKER.
void Simulator::createOrDeleteRobots(){
    if (!new_robots_needed_) return;

    std::vector<std::shared_ptr<Robot>> robots_to_create{};
    std::vector<std::shared_ptr<Robot>> robots_to_delete{};
    Eigen::VectorXd starting, turning, ending; // Waypoints : [x,y,xdot,ydot].

    if (globals.FORMATION == "Payload") {
        new_robots_needed_ = false;
        float robot_radius = globals.ROBOT_RADIUS;
        
        // 获取payload信息createOrDelete
        if (payloads_.empty()) return;
        
        auto payload = payloads_.begin()->second;

        auto [contact_points, contact_normals] = payload->getContactPointsAndNormals();
        Eigen::Vector2d payload_centroid = payload->getPosition();
        Eigen::Vector2d payload_target = payload->getTarget();
        Eigen::MatrixXd payload_R = Quat2Rot(payload->getTargetRotation());

        std::cout << "Rotation matrix is" << payload_R << std::endl;
        
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

            Eigen::Vector2d dir = (contact_point - payload_centroid).normalized();


            // 机器人初始位置：接触点 - 机器人半径 * 法向量
            Eigen::Vector2d robot_start_pos = contact_point;
            
            Eigen::VectorXd starting(4);
            starting << robot_start_pos.x(), robot_start_pos.y(), 0.0, 0.0;

            Eigen::Vector2d relative_r = payload_R * (contact_point - payload_centroid);

            Eigen::VectorXd ending(4);

            ending << payload_target.x() + relative_r.x(), 
                    payload_target.y() + relative_r.y(),
                    0.0, 0.0;
            // store the first 2 elements in ending to simulator::endings
            endings_.push_back(Eigen::Vector2d(ending(0), ending(1)));

            std::deque<Eigen::VectorXd> waypoints{starting, ending};
            
            Color robot_color = ColorFromHSV(contact_point_index * 360.0f / contact_points.size(), 1.0f, 0.75f);
            
            auto robot = std::make_shared<Robot>(this, next_rid_++, waypoints, robot_radius, robot_color, getPhysicsWorld());
            
            // 分配接触点索引
            robot->assigned_contact_point_index_ = contact_point_index;
            
            robots_to_create.push_back(robot);
            
            std::cout << "Robot " << robot->rid_ << " assigned to contact point " << contact_point_index 
                      << " at position (" << contact_point.x() << ", " << contact_point.y() << ")" << std::endl;
        }
        
    } else if (globals.FORMATION == "Test"){
        new_robots_needed_ = false;
        float robot_radius = globals.ROBOT_RADIUS;
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
                }
            }
        }
    }

    locateNearbys(robots_);
    // Geometry factors removed - only dynamics, interrobot, and obstacle factors are used
    // Delete robots
    for (auto robot : robots_to_delete){
        deleteRobot(robot);
    }
}

/*******************************************************************************/
// Create GTSAM robots for centralized optimization approach (Payload formation only)
/*******************************************************************************/
void Simulator::createOrDeleteRobotsGTSAM(){
    if (!new_robots_needed_) return;

    if (globals.FORMATION == "Payload") {
        new_robots_needed_ = false;
        float robot_radius = globals.ROBOT_RADIUS;
        
        // Get payload information
        if (payloads_.empty()) return;
        
        auto payload = payloads_.begin()->second;
        auto [contact_points, contact_normals] = payload->getContactPointsAndNormals();
        Eigen::Vector2d payload_centroid = payload->getPosition();
        Eigen::Vector2d payload_target = payload->getTarget();
        Eigen::MatrixXd payload_R = Quat2Rot(payload->getTargetRotation());

        if (contact_points.empty()) {
            std::cout << "Warning: No contact points available for payload" << std::endl;
            return;
        }
        
        // Create GTSAM robots for each contact point
        for (int i = 0; i < globals.NUM_ROBOTS; i++) {
            int contact_point_index = i % contact_points.size();
            
            // Get corresponding contact point and normal
            Eigen::Vector2d contact_point = contact_points[contact_point_index];
            Eigen::Vector2d contact_normal = contact_normals[contact_point_index];

            // Calculate contact point in payload frame (r_i) - matching Robot.cpp logic
            Eigen::Vector2d r_i = contact_point - payload_centroid;
            
            // Robot initial position at contact point
            Eigen::Vector2d robot_start_pos = contact_point;
            
            // Define start state [x, y, xdot, ydot]
            gtsam::Vector4 start_state;
            start_state << robot_start_pos.x(), robot_start_pos.y(), 0.0, 0.0;

            // Calculate target position using same logic as GBP
            Eigen::Vector2d relative_r = payload_R * (contact_point - payload_centroid);
            
            // Define target state [x, y, xdot, ydot] 
            gtsam::Vector4 target_state;
            target_state << payload_target.x() + relative_r.x(), 
                           payload_target.y() + relative_r.y(),
                           0.0, 0.0;

            // Create robot color based on contact point index
            Color robot_color = ColorFromHSV(contact_point_index * 360.0f / contact_points.size(), 1.0f, 0.75f);
            
            // Create GTSAM robot with payload coupling support
            auto robot_gtsam = std::make_shared<RobotGTSAM>(start_state, target_state, this, robot_color, robot_radius, getPhysicsWorld(), payload.get(), r_i);
            
            robots_gtsam_[next_rid_++] = robot_gtsam;
            
            std::cout << "GTSAM Robot " << (next_rid_-1) << " created for contact point " << contact_point_index 
                      << " at position (" << contact_point.x() << ", " << contact_point.y() << ")" << std::endl;
        }
        
        // Optimize all GTSAM robots
        for (auto& robot_pair : robots_gtsam_) {
            robot_pair.second->updateCurrent();
        }
    }
}

/*******************************************************************************/
// GTSAM timestep implementation - centralized optimization approach
/*******************************************************************************/
void Simulator::timestepGTSAM() {
    // Physics sync before optimization
    syncGTSAMPhysicsToLogical();
    
    // Core GTSAM operations  
    updateDistributedPayloadControlGTSAM();
    optimizeAllGTSAMRobots();
    updateGTSAMRobotStates();
    
    // Physics sync after optimization
    syncGTSAMLogicalToPhysics();
    
    // Physics world step (if physics enabled)
    if (physicsWorld_) {
        physicsWorld_->Step(globals.TIMESTEP, 6, 2);
    }
    
    // Update payload states from physics (matching existing timestep)
    for (auto& [pid, payload] : payloads_) {
        payload->update();
    }
    
    // Export payload trajectory data to CSV
    exportPayloadTrajectory();
    
    clock_++;
    if (clock_ >= globals.MAX_TIME) globals.RUN = false;
}

void Simulator::syncGTSAMPhysicsToLogical() {
    for (auto& [rid, robot_gtsam] : robots_gtsam_) {
        robot_gtsam->syncPhysicsToLogical();
    }
}

void Simulator::syncGTSAMLogicalToPhysics() {
    for (auto& [rid, robot_gtsam] : robots_gtsam_) {
        robot_gtsam->syncLogicalToPhysics();
    }
}

void Simulator::optimizeAllGTSAMRobots() {
    for (auto& [rid, robot_gtsam] : robots_gtsam_) {
        robot_gtsam->updateCurrent();
    }
}

void Simulator::updateGTSAMRobotStates() {
    // Update horizon state for all GTSAM robots (matching Robot.cpp interface)
    for (auto& [rid, robot_gtsam] : robots_gtsam_) {
        robot_gtsam->updateHorizon();
    }
}

void Simulator::updateDistributedPayloadControlGTSAM() {
    // Adapted version of updateDistributedPayloadControl for GTSAM robots
    if (globals.FORMATION != "Payload" || payloads_.empty()) return;
    
    auto payload = payloads_.begin()->second;
    auto [contact_points, contact_normals] = payload->getContactPointsAndNormals();
    
    if (contact_points.empty()) return;
    
    // Update payload contact assignments for GTSAM robots
    for (auto& [rid, robot_gtsam] : robots_gtsam_) {
        // For now, we'll keep the same simple assignment as in createOrDeleteRobotsGTSAM
        // More sophisticated contact assignment can be added later
        
        // Enable/disable rigid attachment for payload transport if configured
        if (globals.USE_RIGID_ATTACHMENT) {
            // The attachment was already set up in createOrDeleteRobotsGTSAM
            // Here we could update attachment points if needed
        }
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

/*******************************************************************************/
// Load obstacles from JSON configuration file
/*******************************************************************************/
void Simulator::loadObstacles() {
    std::cout << "Loading obstacles from: " << globals.OBSTACLE_CONFIG_FILE << std::endl;
    
    if (globals.OBSTACLE_CONFIG_FILE.empty()) {
        std::cout << "No obstacle configuration file specified." << std::endl;
        return;
    }
    
    std::ifstream file(globals.OBSTACLE_CONFIG_FILE);
    if (!file.is_open()) {
        std::cout << "Warning: Could not open obstacle configuration file: " << globals.OBSTACLE_CONFIG_FILE << std::endl;
        return;
    }
    
    nlohmann::json json_data;
    try {
        file >> json_data;
        
        if (json_data.contains("obstacles")) {
            for (const auto& obstacle_json : json_data["obstacles"]) {
                ObstacleData obstacle;
                obstacle.id = obstacle_json["id"];
                obstacle.position = Eigen::Vector2d(obstacle_json["position"]["x"], obstacle_json["position"]["y"]);
                obstacle.radius = obstacle_json["radius"];
                
                obstacles_.push_back(obstacle);
                std::cout << "Loaded obstacle " << obstacle.id << " at position (" 
                          << obstacle.position.x() << ", " << obstacle.position.y() 
                          << ") with radius " << obstacle.radius << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cout << "Error parsing obstacle configuration file: " << e.what() << std::endl;
    }
}

/*******************************************************************************/
// Draw all obstacles as cylinders
/*******************************************************************************/
void Simulator::drawObstacles() {
    if (!globals.DISPLAY) return;
    
    static bool debug_printed = false;
    if (!debug_printed) {
        std::cout << "Drawing " << obstacles_.size() << " obstacles" << std::endl;
        debug_printed = true;
    }
    
    for (const auto& obstacle : obstacles_) {
        float x = static_cast<float>(obstacle.position.x());
        float y = static_cast<float>(obstacle.position.y());
        float radius = obstacle.radius;
        float height = 8.0f; // Make obstacles taller for better visibility
        
        Vector3 position3D = {x, height / 2.0f, y};
        
        // Draw cylinder for obstacle with bright red color for visibility
        DrawCylinder(position3D, radius, radius, height, 16, RED);
        DrawCylinderWires(position3D, radius, radius, height, 16, BLACK);
    }
}

void Simulator::initCSVExport() {
    trajectory_csv_file_.open("payload_trajectory.csv");
    if (trajectory_csv_file_.is_open()) {
        // Write header row
        trajectory_csv_file_ << "payload_x,payload_y,payload_orientation,velocity_x,velocity_y,gbp_duration_us" << std::endl;
        std::cout << "CSV export initialized: payload_trajectory.csv" << std::endl;
    } else {
        std::cerr << "Error: Could not create CSV file for trajectory export" << std::endl;
    }
}

void Simulator::exportPayloadTrajectory() {
    if (!globals.EXPORT_TRAJECTORY_DATA || !trajectory_csv_file_.is_open()) {
        return;
    }
    
    if (payloads_.empty()) {
        return;
    }
    
    // Get the first (and likely only) payload
    auto payload = payloads_.begin()->second;
    
    // Write payload data to CSV
    trajectory_csv_file_ << payload->position_.x() << ","
                        << payload->position_.y() << ","
                        << payload->rotation_ << ","
                        << payload->velocity_.x() << ","
                        << payload->velocity_.y() << ","
                        << gbp_duration_microseconds_ << std::endl;
    
    // Flush to ensure data is written immediately
    trajectory_csv_file_.flush();
}