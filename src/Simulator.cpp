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
#include <fstream>
#include <sstream>
#include <algorithm>


/*******************************************************************************/
// Raylib setup
/*******************************************************************************/
Simulator::Simulator(){
    SetTraceLogLevel(LOG_ERROR);  
    if (globals.DISPLAY) {
        SetTargetFPS(60);
        InitWindow(globals.SCREEN_SZ, globals.SCREEN_SZ, globals.WINDOW_TITLE);
    }
    treeOfRobots_ = new KDTree(2, robot_positions_, 50); 

    // For display only
    // User inputs an obstacle image where the obstacles are BLACK and background is WHITE.
    obstacleImg = LoadImage(globals.OBSTACLE_FILE.c_str());
    if (obstacleImg.width==0) obstacleImg = GenImageColor(globals.WORLD_SZ, globals.WORLD_SZ, WHITE);

    // However for calculation purposes the image needs to be inverted.
    ImageColorInvert(&obstacleImg);
    graphics = new Graphics(obstacleImg);

    if (!initializeMuJoCo()) {
        std::cerr << "Failed to initialize MuJoCo" << std::endl;
    }

    if (globals.FORMATION == "Payload"){
        createPayload(Eigen::Vector2d(0., 0.), globals.PAYLOAD_WIDTH, globals.PAYLOAD_HEIGHT);
    }
};



void Simulator::createPayload(Eigen::Vector2d position, float width, float height) {
    auto payload = std::make_shared<Payload>(this, next_payload_id_++, position, width, height, globals.PAYLOAD_DENSITY);
    payloads_[payload->payload_id_] = payload;
    
    // 设置目标位置
    Eigen::Vector2d absolute_target_position = position + Eigen::Vector2d(globals.TARGET_RELATIVE_X, globals.TARGET_RELATIVE_Y);
    payload->setTarget(absolute_target_position);
    
    // 设置目标朝向
    if (std::abs(globals.TARGET_RELATIVE_ROTATION) > 0.001) {
        payload->setTargetFromRelativeRotation(globals.TARGET_RELATIVE_ROTATION);
        std::cout << "Payload target: position(" << absolute_target_position.x() << ", " << absolute_target_position.y() 
                  << ") relative_rotation(" << globals.TARGET_RELATIVE_ROTATION << " rad)" << std::endl;
    } else {
        payload->target_orientation_ = payload->initial_orientation_;
        std::cout << "Payload target: position only, no rotation" << std::endl;
    }
    
    // 标记需要重建模型
    model_needs_rebuild_ = true;
}

/*******************************************************************************/
// Destructor
/*******************************************************************************/
Simulator::~Simulator(){
    delete treeOfRobots_;
    int n = robots_.size();
    for (int i = 0; i < n; ++i) robots_.erase(i);
    destroyMuJoCo();

    if (globals.DISPLAY) {
        delete graphics;
        CloseWindow();
    }

};

bool Simulator::initializeMuJoCo() {

    // 生成基本的XML模型
    std::string xml_content = generateMuJoCoXML();
    
    return loadMuJoCoModel(xml_content);
}

bool Simulator::loadMuJoCoModel(const std::string& xml_content) {
    char error[1000] = "Could not load XML model";
    
    // 创建临时文件
    std::string temp_filename = "/tmp/mujoco_model_" + std::to_string(clock_) + ".xml";
    std::ofstream temp_file(temp_filename);
    if (!temp_file.is_open()) {
        std::cerr << "Failed to create temp XML file" << std::endl;
        return false;
    }
    
    temp_file << xml_content;
    temp_file.close();
    
    // 销毁现有模型
    if (mujoco_data_) mj_deleteData(mujoco_data_);
    if (mujoco_model_) mj_deleteModel(mujoco_model_);
    
    // 加载新模型
    mujoco_model_ = mj_loadXML(temp_filename.c_str(), nullptr, error, 1000);
    std::remove(temp_filename.c_str());
    
    if (!mujoco_model_) {
        std::cerr << "MuJoCo model loading failed: " << error << std::endl;
        return false;
    }
    
    // 创建数据
    mujoco_data_ = mj_makeData(mujoco_model_);
    if (!mujoco_data_) {
        std::cerr << "Failed to create MuJoCo data" << std::endl;
        mj_deleteModel(mujoco_model_);
        mujoco_model_ = nullptr;
        return false;
    }
    
    // 初始化可视化（如果需要）
    if (globals.DISPLAY) {
        initializeMuJoCoVisualization();
    }
    
    std::cout << "MuJoCo model loaded successfully with " << mujoco_model_->nbody << " bodies" << std::endl;
    return true;
}

std::string Simulator::generateMuJoCoXML() {
    std::ostringstream xml;
    
    xml << R"(<?xml version="1.0" encoding="utf-8"?>
<mujoco model="robot_payload_collaboration">
    <compiler angle="radian" autolimits="true"/>
    <option timestep=")" << globals.MUJOCO_TIMESTEP << R"(" gravity="0 0 0"/>
    
    <default>
        <geom rgba="0.8 0.6 0.4 1" friction="0.5 0.1 0.1"/>
        <joint damping="0.1"/>
        <default class="robot">
            <geom rgba="0.2 0.8 0.2 1" type="cylinder"/>
        </default>
        <default class="payload">
            <geom rgba="0.8 0.2 0.2 1" type="box"/>
        </default>
    </default>
    
    <asset>
        <material name="grid" texture="grid" texrepeat="8 8" texuniform="true"/>
        <texture name="grid" type="2d" builtin="checker" width="512" height="512" 
                 rgb1="0.1 0.2 0.3" rgb2="0.2 0.3 0.4"/>
    </asset>
    
    <worldbody>
        <geom name="floor" type="plane" size=")" << globals.WORLD_SZ/2 << " " << globals.WORLD_SZ/2 << R"( 0.1" material="grid"/>
        <light diffuse=".8 .8 .8" pos="0 0 4" dir="0 0 -1"/>
        
        <!-- 这里会动态添加机器人和载荷 -->
)";

    // 添加现有机器人
    for (const auto& [rid, robot] : robots_) {
        xml << generateRobotXML(rid, Eigen::Vector2d(robot->position_(0), robot->position_(1)), robot->robot_radius_);
    }
    
    // 添加现有载荷
    for (const auto& [pid, payload] : payloads_) {
        xml << generatePayloadXML(pid, payload->position_, payload->width_, payload->height_);
    }
    
    xml << R"(    </worldbody>
</mujoco>)";
    
    return xml.str();
}

std::string Simulator::generateRobotXML(int robot_id, const Eigen::Vector2d& pos, double radius) {
    std::ostringstream xml;
    xml << R"(        <body name="robot_)" << robot_id << R"(" pos=")" << pos.x() << " " << pos.y() << R"( 0.1">)"
        << R"(<freejoint name="robot_joint_)" << robot_id << R"("/>)"
        << R"(<geom name="robot_geom_)" << robot_id << R"(" class="robot" size=")" << radius << R"( 0.1"/>)"
        << R"(</body>)" << std::endl;
    return xml.str();
}

std::string Simulator::generatePayloadXML(int payload_id, const Eigen::Vector2d& pos, double width, double height) {
    std::ostringstream xml;
    xml << R"(        <body name="payload_)" << payload_id << R"(" pos=")" << pos.x() << " " << pos.y() << R"( 0.5">)"
        << R"(<freejoint name="payload_joint_)" << payload_id << R"("/>)"
        << R"(<geom name="payload_geom_)" << payload_id << R"(" class="payload" size=")" 
        << width/2 << " " << height/2 << R"( 0.5"/>)"
        << R"(</body>)" << std::endl;
    return xml.str();
}

void Simulator::destroyMuJoCo() {
    if (mujoco_data_) {
        mj_deleteData(mujoco_data_);
        mujoco_data_ = nullptr;
    }
    
    if (mujoco_model_) {
        mj_deleteModel(mujoco_model_);
        mujoco_model_ = nullptr;
    }
    
    if (mujoco_window_) {
        glfwDestroyWindow(mujoco_window_);
        mujoco_window_ = nullptr;
        glfwTerminate();
    }
}

void Simulator::stepMuJoCo() {
    if (!mujoco_model_ || !mujoco_data_) return;
    
    // 执行MuJoCo物理步进
    mj_step(mujoco_model_, mujoco_data_);
}

void Simulator::renderMuJoCo() {
    if (!mujoco_window_ || !mujoco_model_ || !mujoco_data_) return;
    
    // 获取窗口尺寸
    int width, height;
    glfwGetFramebufferSize(mujoco_window_, &width, &height);
    mjrRect viewport = {0, 0, width, height};
    
    // 更新场景
    mjv_updateScene(mujoco_model_, mujoco_data_, &mujoco_opt_, nullptr, 
                    &mujoco_cam_, mjCAT_ALL, &mujoco_scn_);
    
    // 渲染
    mjr_render(viewport, &mujoco_scn_, &mujoco_con_);
    
    // 交换缓冲区
    glfwSwapBuffers(mujoco_window_);
    glfwPollEvents();
}


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
    auto robot_it = robots_.find(robot_id);
    auto payload_it = payloads_.find(payload_id);
    
    if (robot_it == robots_.end() || payload_it == payloads_.end()) {
        return false;
    }
    
    auto robot_mujoco_it = robot_to_mujoco_id_.find(robot_id);
    auto payload_mujoco_it = payload_to_mujoco_id_.find(payload_id);
    
    if (robot_mujoco_it == robot_to_mujoco_id_.end() || 
        payload_mujoco_it == payload_to_mujoco_id_.end()) {
        return false;
    }
    
    return areInContactMuJoCo(robot_mujoco_it->second, payload_mujoco_it->second);
}

bool Simulator::areInContactMuJoCo(int body1_id, int body2_id) const {
    if (!mujoco_model_ || !mujoco_data_) return false;
    
    // 检查MuJoCo接触列表
    for (int i = 0; i < mujoco_data_->ncon; i++) {
        mjContact& con = mujoco_data_->contact[i];
        
        // 获取接触涉及的body ID
        int geom1_body = mujoco_model_->geom_bodyid[con.geom1];
        int geom2_body = mujoco_model_->geom_bodyid[con.geom2];
        
        // 检查是否是我们关心的两个body之间的接触
        if ((geom1_body == body1_id && geom2_body == body2_id) ||
            (geom1_body == body2_id && geom2_body == body1_id)) {
            
            // 检查接触距离（负值表示穿透，即真实接触）
            if (con.dist <= 0) {
                return true;
            }
        }
    }
    
    return false;
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> Simulator::getContactPoint(int robot_id, int payload_id) {
    auto robot_mujoco_it = robot_to_mujoco_id_.find(robot_id);
    auto payload_mujoco_it = payload_to_mujoco_id_.find(payload_id);
    
    if (robot_mujoco_it == robot_to_mujoco_id_.end() || 
        payload_mujoco_it == payload_to_mujoco_id_.end()) {
        return {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
    }
    
    return getContactInfoMuJoCo(robot_mujoco_it->second, payload_mujoco_it->second);
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> Simulator::getContactInfoMuJoCo(int body1_id, int body2_id) const {
    if (!mujoco_model_ || !mujoco_data_) {
        return {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
    }
    
    // 查找两个body之间的接触
    for (int i = 0; i < mujoco_data_->ncon; i++) {
        mjContact& con = mujoco_data_->contact[i];
        
        int geom1_body = mujoco_model_->geom_bodyid[con.geom1];
        int geom2_body = mujoco_model_->geom_bodyid[con.geom2];
        
        if ((geom1_body == body1_id && geom2_body == body2_id) ||
            (geom1_body == body2_id && geom2_body == body1_id)) {
            
            if (con.dist <= 0) {  // 真实接触
                // 接触点位置
                Eigen::Vector2d contact_point(con.pos[0], con.pos[1]);
                
                // 接触法向量（从body1指向body2）
                Eigen::Vector2d contact_normal(con.frame[0], con.frame[1]);
                
                // 确保法向量方向正确（从body1指向body2）
                if (geom1_body == body2_id && geom2_body == body1_id) {
                    contact_normal = -contact_normal;
                }
                
                return {contact_point, contact_normal};
            }
        }
    }
    
    return {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
}

void Simulator::applyForcesToPayloadMuJoCo(std::shared_ptr<Payload> payload, 
                                          const Eigen::VectorXd& forces,
                                          const std::vector<Eigen::Vector2d>& contact_points,
                                          const std::vector<Eigen::Vector2d>& contact_normals) {
    if (payload->mujoco_body_id_ < 0) return;
    
    // 清零当前施加的力
    for (int i = 0; i < 6; i++) {
        mujoco_data_->xfrc_applied[payload->mujoco_body_id_ * 6 + i] = 0.0;
    }
    
    // 遍历所有接触点，施加对应的力
    for (int i = 0; i < forces.size() && i < contact_points.size() && i < contact_normals.size(); i++) {
        if (forces(i) > 0.01) { // 只施加显著的力
            // 计算力向量
            Eigen::Vector2d force_vector = forces(i) * contact_normals[i];
            
            // 计算相对于payload质心的力矩
            Eigen::Vector2d payload_center = payload->getMuJoCoPosition();
            Eigen::Vector2d r = contact_points[i] - payload_center;
            double torque = r.x() * force_vector.y() - r.y() * force_vector.x();
            
            // 累加到总的外力和力矩
            mujoco_data_->xfrc_applied[payload->mujoco_body_id_ * 6 + 0] += force_vector.x(); // fx
            mujoco_data_->xfrc_applied[payload->mujoco_body_id_ * 6 + 1] += force_vector.y(); // fy
            mujoco_data_->xfrc_applied[payload->mujoco_body_id_ * 6 + 5] += torque;           // τz
        }
    }
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
    // 直接调用MuJoCo版本
    applyForcesToPayloadMuJoCo(payload, forces, contact_points, contact_normals);
}

bool Simulator::initializeMuJoCoVisualization() {
    // 初始化GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW for MuJoCo visualization" << std::endl;
        return false;
    }
    
    // 创建MuJoCo可视化窗口
    mujoco_window_ = glfwCreateWindow(800, 600, "MuJoCo Physics Visualization", nullptr, nullptr);
    if (!mujoco_window_) {
        std::cerr << "Failed to create MuJoCo window" << std::endl;
        glfwTerminate();
        return false;
    }
    
    glfwMakeContextCurrent(mujoco_window_);
    
    // 初始化MuJoCo可视化组件
    mjv_defaultCamera(&mujoco_cam_);
    mjv_defaultOption(&mujoco_opt_);
    mjv_defaultScene(&mujoco_scn_);
    mjr_defaultContext(&mujoco_con_);
    
    // 设置场景
    mjv_makeScene(mujoco_model_, &mujoco_scn_, 2000);
    mjr_makeContext(mujoco_model_, &mujoco_con_, mjFONTSCALE_150);
    
    // 设置相机位置
    mujoco_cam_.azimuth = 90;
    mujoco_cam_.elevation = -45;
    mujoco_cam_.distance = 50;
    mujoco_cam_.lookat[0] = 0;
    mujoco_cam_.lookat[1] = 0;
    mujoco_cam_.lookat[2] = 0;
    
    std::cout << "MuJoCo visualization initialized" << std::endl;
    return true;
}

void Simulator::applyDirectPayloadVelocityControl() {
    for (auto& [pid, payload] : payloads_) {
        if (payload->mujoco_body_id_ < 0) continue;
        
        // 计算期望的线速度和角速度
        Eigen::Vector2d desired_velocity = computeDesiredPayloadVelocity(payload);
        double desired_angular_velocity = computeDesiredPayloadAngularVelocity(payload);
        
        // 直接设置payload的速度（Box2D提供的功能）
        payload->setMuJoCoVelocity(desired_velocity);
        payload->setMuJoCoAngularVelocity(desired_angular_velocity);
        
        // 调试输出
        static int debug_counter = 0;
        if (debug_counter++ % 60 == 0) {  // 每60帧输出一次
            Eigen::Vector2d current_velocity = payload->getMuJoCoVelocity();
            double current_angular_velocity = payload->getMuJoCoAngularVelocity();
            
            std::cout << "Direct velocity control - Set vel: (" << desired_velocity.x() << ", " << desired_velocity.y() 
                        << "), Actual vel: (" << current_velocity.x() << ", " << current_velocity.y() << ")" << std::endl;
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


void Simulator::timestep() {
    if (globals.SIM_MODE != Timestep) return;
    
    // ========== 1. 控制方法选择 ==========
    // 根据配置选择不同的payload控制方法
    if (globals.USE_FORCE_ALLOCATION && globals.FORMATION == "Payload") {
        // 基于因子图的力分配控制
        updateForceAllocationSystem();
        
        // 计算接触点几何参数（如果需要）
        for (auto [r_id, robot] : robots_) {
            robot->updateContactForceGeometry();
        }
        
    } else if (globals.USE_DIRECT_PAYLOAD_VELOCITY) {
        // 直接速度控制（绕过因子图）
        applyDirectPayloadVelocityControl();
        
    } else if (globals.USE_DISTRIBUTED_PAYLOAD_CONTROL) {
        // 使用PayloadTwistFactor的分布式控制
        updateDistributedPayloadControl();
        
    } else {
        // 默认：集中式最小二乘力分配
        computeLeastSquares();
    }
    
    // ========== 2. 机器人邻居关系计算 ==========
    calculateRobotNeighbours(robots_);
    
    // ========== 3. 因子几何参数更新（一次性设置）==========
    static bool geometry_updated = false;
    if ((globals.USE_DISTRIBUTED_PAYLOAD_CONTROL || globals.USE_FORCE_ALLOCATION) && 
        !globals.USE_DIRECT_PAYLOAD_VELOCITY && !geometry_updated) {
        
        for (auto [r_id, robot] : robots_) {
            // 更新PayloadTwistFactor几何参数
            if (globals.USE_DISTRIBUTED_PAYLOAD_CONTROL) {
                robot->updatePayloadFactorGeometry();
            }
            
            // 更新ForceAllocationFactor几何参数
            if (globals.USE_FORCE_ALLOCATION) {
                robot->updateContactForceGeometry();
            }
        }
        geometry_updated = true;
        std::cout << "Factor geometry parameters updated" << std::endl;
    }
    
    // ========== 4. 因子图连接更新 ==========
    if ((globals.USE_DISTRIBUTED_PAYLOAD_CONTROL || globals.USE_FORCE_ALLOCATION) && 
        !globals.USE_DIRECT_PAYLOAD_VELOCITY) {
        
        for (auto [r_id, robot] : robots_) {
            // 更新机器人间因子（避免碰撞等）
            robot->updateInterrobotFactors();
            
            // 如果使用力分配，可能需要更新力分配相关的连接
            // （通常ForceAllocationFactor在创建时就建立了所有必要连接）
        }
    }
    
    // ========== 5. 通信故障模拟 ==========
    setCommsFailure(globals.COMMS_FAILURE_RATE);
    
    // ========== 6. 因子图优化（GBP迭代）==========
    if ((globals.USE_DISTRIBUTED_PAYLOAD_CONTROL || globals.USE_FORCE_ALLOCATION) && 
        !globals.USE_DIRECT_PAYLOAD_VELOCITY) {
        
        // 执行多轮GBP迭代
        for (int i = 0; i < globals.NUM_ITERS; i++) {
            // 内部迭代：机器人内部的变量和因子
            iterateGBP(1, INTERNAL, robots_);
            
            // 外部迭代：机器人间的协调因子
            iterateGBP(1, EXTERNAL, robots_);
        }
        
        // ========== 7. 机器人状态更新 ==========
        for (auto [r_id, robot] : robots_) {
            // 更新地平线状态（目标导向）
            robot->updateHorizon();
            
            // 更新当前状态（执行计划的第一步）
            robot->updateCurrent();
        }
        
        // 可选：更新payload变量的先验（如果使用PayloadTwistFactor）
        if (globals.USE_DISTRIBUTED_PAYLOAD_CONTROL) {
            for (auto [r_id, robot] : robots_) {
                robot->updatePayloadTwistPriors();
            }
        }
    }
    
    // ========== 8. MUJOCO物理世界更新 ==========
    stepMuJoCo();  // MuJoCo物理步进
    
    // 同步物理状态到逻辑状态
    for (auto [r_id, robot] : robots_) {
        robot->syncPhysicsToLogical();  // 内部调用syncFromMuJoCo()
    }
    
    // 更新payload状态
    for (auto [p_id, payload] : payloads_) {
        payload->update();  // 内部调用syncFromMuJoCo()
    }
    
    // ========== 9. 调试信息输出 ==========
    static int debug_counter = 0;
    if (debug_counter++ % 120 == 0) {  // 每2秒输出一次
        std::cout << "\n=== Timestep " << clock_ << " Debug Info ===" << std::endl;
        
        if (globals.USE_FORCE_ALLOCATION && !payloads_.empty()) {
            // 输出力分配信息
            auto payload = payloads_.begin()->second;
            std::cout << "Payload position: (" << payload->getPosition().transpose() << ")" << std::endl;
            std::cout << "Target position: (" << payload->target_position_.transpose() << ")" << std::endl;
            
            // 输出各机器人的接触力
            for (auto [r_id, robot] : robots_) {
                if (!robot->contact_force_variables_.empty()) {
                    double contact_force = robot->contact_force_variables_[0]->mu_(0);
                    std::cout << "Robot " << r_id << " contact force: " << contact_force << std::endl;
                }
            }
        }
        
        if (globals.USE_DISTRIBUTED_PAYLOAD_CONTROL) {
            // 输出payload twist信息
            for (auto [r_id, robot] : robots_) {
                if (!robot->payload_twist_variables_.empty()) {
                    auto twist_var = robot->payload_twist_variables_[0];
                    std::cout << "Robot " << r_id << " payload twist: [" 
                              << twist_var->mu_.transpose() << "]" << std::endl;
                }
            }
        }
        
        std::cout << "Control mode: ";
        if (globals.USE_FORCE_ALLOCATION) std::cout << "Force Allocation";
        else if (globals.USE_DIRECT_PAYLOAD_VELOCITY) std::cout << "Direct Velocity";
        else if (globals.USE_DISTRIBUTED_PAYLOAD_CONTROL) std::cout << "Distributed Twist";
        else std::cout << "Centralized Least Squares";
        std::cout << std::endl;
        
        // 输出当前使用的物理引擎
        std::cout << "Physics engine: MuJoCo" << std::endl;
    }
    
    // ========== 10. 任务完成检查 ==========
    if (globals.FORMATION == "Payload" && !payloads_.empty()) {
        auto payload = payloads_.begin()->second;
        
        // 检查是否到达目标
        Eigen::Vector2d position_error = payload->target_position_ - payload->position_;
        double rotation_error = std::abs(payload->getRotationError());
        
        if (position_error.norm() < 0.5 && rotation_error < 0.1) {
            payload->task_completed_ = true;
            
            static bool completion_logged = false;
            if (!completion_logged) {
                std::cout << "\n🎉 Payload manipulation task completed! 🎉" << std::endl;
                std::cout << "Final position error: " << position_error.norm() << std::endl;
                std::cout << "Final rotation error: " << rotation_error << " rad" << std::endl;
                completion_logged = true;
            }
        }
    }
    
    // ========== 11. 时间推进和终止条件 ==========
    clock_++;
    
    if (clock_ >= globals.MAX_TIME) {
        std::cout << "\nSimulation completed after " << globals.MAX_TIME << " timesteps." << std::endl;
        globals.RUN = false;
    }
    
    // 可选：基于任务完成情况的早期终止
    if (globals.FORMATION == "Payload" && !payloads_.empty()) {
        auto payload = payloads_.begin()->second;
        if (payload->task_completed_ && clock_ > 100) {  // 至少运行100步
            std::cout << "\nTask completed! Ending simulation early." << std::endl;
            globals.RUN = false;
        }
    }
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

/*******************************************************************************/
// Create new robots if needed. Handles deletion of robots out of bounds. 
// New formations must modify the vectors "robots to create" and optionally "robots_to_delete"
// by appending (push_back()) a shared pointer to a Robot class.
/*******************************************************************************/
void Simulator::createOrDeleteRobots(){
    if (!new_robots_needed_) return;

    std::vector<std::shared_ptr<Robot>> robots_to_create{};
    
    // 只保留Payload formation
    if (globals.FORMATION == "Payload") {
        new_robots_needed_ = false;
        
        if (payloads_.empty()) return;
        
        auto payload = payloads_.begin()->second;
        auto [contact_points, contact_normals] = payload->getContactPointsAndNormals();
        
        if (contact_points.empty()) return;
        
        // 创建机器人
        for (int i = 0; i < globals.NUM_ROBOTS; i++) {
            int contact_point_index = i % contact_points.size();
            Eigen::Vector2d contact_point = contact_points[contact_point_index];
            Eigen::Vector2d contact_normal = contact_normals[contact_point_index];
            
            // 机器人初始位置
            Eigen::Vector2d robot_start_pos = contact_point - globals.ROBOT_RADIUS * contact_normal;
            
            Eigen::VectorXd starting(4);
            starting << robot_start_pos.x(), robot_start_pos.y(), 0.0, 0.0;
            
            std::deque<Eigen::VectorXd> waypoints{starting, starting};  // 起点=终点
            Color robot_color = ColorFromHSV(contact_point_index * 90.0f, 1.0f, 0.75f);  // 简化颜色
            
            auto robot = std::make_shared<Robot>(this, next_rid_++, waypoints, globals.ROBOT_RADIUS, robot_color);
            robot->assigned_contact_point_index_ = contact_point_index;
            robots_to_create.push_back(robot);
        }
    }
    
    // 添加机器人到系统
    for (auto robot : robots_to_create){
        robot_positions_[robot->rid_] = std::vector<double>{robot->waypoints_[0](0), robot->waypoints_[0](1)};
        robots_[robot->rid_] = robot;
        model_needs_rebuild_ = true;
    }
    
    // 重建MuJoCo模型
    if (model_needs_rebuild_) {
        rebuildMuJoCoModel();
        model_needs_rebuild_ = false;
    }
}

bool Simulator::rebuildMuJoCoModel() {
    std::cout << "Rebuilding MuJoCo model with " << robots_.size() << " robots and " 
              << payloads_.size() << " payloads..." << std::endl;
    
    // 重新生成XML并加载模型
    std::string xml_content = generateMuJoCoXML();
    bool success = loadMuJoCoModel(xml_content);
    
    if (success) {
        updateEntityReferences();
    }
    
    return success;
}

void Simulator::updateEntityReferences() {
    // 清空映射
    robot_to_mujoco_id_.clear();
    payload_to_mujoco_id_.clear();
    mujoco_to_robot_id_.clear();
    mujoco_to_payload_id_.clear();
    
    // 为机器人建立ID映射
    for (auto& [rid, robot] : robots_) {
        std::string robot_name = "robot_" + std::to_string(rid);
        int mujoco_id = mj_name2id(mujoco_model_, mjOBJ_BODY, robot_name.c_str());
        
        if (mujoco_id >= 0) {
            robot_to_mujoco_id_[rid] = mujoco_id;
            mujoco_to_robot_id_[mujoco_id] = rid;
            robot->mujoco_body_id_ = mujoco_id;
            robot->setMuJoCoReferences(mujoco_model_, mujoco_data_);
            
            std::cout << "Robot " << rid << " mapped to MuJoCo body " << mujoco_id << std::endl;
        } else {
            std::cerr << "Failed to find MuJoCo body for robot " << rid << std::endl;
        }
    }
    
    // 为载荷建立ID映射
    for (auto& [pid, payload] : payloads_) {
        std::string payload_name = "payload_" + std::to_string(pid);
        int mujoco_id = mj_name2id(mujoco_model_, mjOBJ_BODY, payload_name.c_str());
        
        if (mujoco_id >= 0) {
            payload_to_mujoco_id_[pid] = mujoco_id;
            mujoco_to_payload_id_[mujoco_id] = pid;
            payload->mujoco_body_id_ = mujoco_id;
            payload->setMuJoCoReferences(mujoco_model_, mujoco_data_);
            
            std::cout << "Payload " << pid << " mapped to MuJoCo body " << mujoco_id << std::endl;
        } else {
            std::cerr << "Failed to find MuJoCo body for payload " << pid << std::endl;
        }
    }
}


void Simulator::createForceAllocationFactors() {
    if (payloads_.empty() || robots_.empty()) return;
    
    auto payload = payloads_.begin()->second;
    auto [contact_points, contact_normals] = payload->getContactPointsAndNormals();
    
    int n_robots = robots_.size();
    int n_contacts = contact_points.size();
    
    std::cout << "Creating force allocation factor: " << n_robots 
              << " robots, " << n_contacts << " contact points" << std::endl;
    
    // 为每个时间步创建一个ForceAllocationFactor
    // 每个因子连接所有机器人在该时间步的接触力变量
    for (int t = 0; t < 15; t++) {  // 假设15个时间步
        std::vector<std::shared_ptr<Variable>> force_variables;
        
        for (auto [rid, robot] : robots_) {
            if (t < robot->contact_force_variables_.size()) {
                force_variables.push_back(robot->contact_force_variables_[t]);
            }
        }
        
        if (force_variables.size() == n_robots) {
            auto force_factor = std::make_shared<ForceAllocationFactor>(
                next_fid_++, -1,  // -1表示系统级因子
                force_variables,
                globals.SIGMA_FACTOR_FORCE_ALLOCATION,  // 需要在Globals中定义
                Eigen::VectorXd::Zero(3),  // 零测量
                contact_points,
                contact_normals
            );
            
            // 将因子添加到第一个机器人的因子图中（或者创建专门的系统级因子图）
            auto first_robot = robots_.begin()->second;
            for (auto var : force_variables) {
                var->add_factor(force_factor);
            }
            first_robot->factors_[force_factor->key_] = force_factor;
            first_robot->force_allocation_factor_ = force_factor;
            
            std::cout << "Created ForceAllocationFactor for timestep " << t << std::endl;
        }
    }
}

void Simulator::updateForceAllocationSystem() {
    if (payloads_.empty()) return;
    
    auto payload = payloads_.begin()->second;
    
    // 计算期望的payload wrench
    Eigen::Vector3d desired_wrench = computeDesiredPayloadWrench(payload);
    
    // 更新所有force allocation factors的期望wrench
    for (auto [rid, robot] : robots_) {
        if (robot->force_allocation_factor_) {
            robot->force_allocation_factor_->updateDesiredWrench(desired_wrench);
        }
    }
    
    static int debug_counter = 0;
    if (debug_counter++ % 60 == 0) {
        std::cout << "Desired wrench: [" << desired_wrench.transpose() << "]" << std::endl;
    }
}

Eigen::Vector3d Simulator::computeDesiredPayloadWrench(std::shared_ptr<Payload> payload) {
    // 类似现有的computeDesiredPayloadVelocity逻辑，但计算所需的力和力矩
    
    Eigen::Vector2d dist_to_target = payload->target_position_ - payload->position_;
    Eigen::Vector2d desired_velocity = Eigen::Vector2d::Zero();
    
    if (dist_to_target.norm() > 0.1) {
        desired_velocity = dist_to_target.normalized() * globals.MAX_SPEED * 0.5;
    }
    
    // 计算期望角速度
    double rotation_error = payload->getRotationError();
    double desired_angular_velocity = 0.0;
    if (std::abs(rotation_error) > 0.01) {
        desired_angular_velocity = std::copysign(1.0, rotation_error) * 
            std::min(static_cast<double>(globals.MAX_ANGULAR_SPEED * 0.5), std::abs(rotation_error));
    }
    
    // 转换为所需的力和力矩
    Eigen::Vector2d current_velocity = payload->getVelocity();
    double current_angular_velocity = payload->getAngularVelocity();
    
    double dt = globals.TIMESTEP;
    double mass = payload->getMass();
    double inertia = payload->getMomentOfInertia();
    
    Eigen::Vector2d required_force = (mass * (desired_velocity - current_velocity)) / dt;
    double required_torque = (inertia * (desired_angular_velocity - current_angular_velocity)) / dt;
    
    return Eigen::Vector3d(required_force.x(), required_force.y(), required_torque);
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