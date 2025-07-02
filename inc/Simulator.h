/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once

#include <map>
#include <memory>
#include <algorithm>
#include <random>

// Core includes
#include <Utils.h>
#include <gbp/GBPCore.h>
#include <gbp/Variable.h>

// Graphics and UI
#include <Graphics.h>
#include <raylib.h>
#include <rlights.h>

// Physics engines
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

// Spatial data structures
#include <nanoflann.h>
#include <KDTreeMapOfVectorsAdaptor.h>

// Project classes
#include "Payload.h"

// Forward declarations
class Robot;
class Graphics;
class TreeOfRobots;
class Payload;

/************************************************************************************/
// The main Simulator. This is where the magic happens.
/************************************************************************************/
class Simulator {
public:
    friend class Robot;
    friend class Factor;
    friend class Payload;

    //==========================================================================
    // CONSTRUCTOR & DESTRUCTOR
    //==========================================================================
    Simulator();
    ~Simulator();

    //==========================================================================
    // CORE SIMULATION DATA
    //==========================================================================
    uint32_t clock_ = 0;                            // Simulation clock (timesteps)
    bool new_robots_needed_ = true;                 // Whether to create new robots dynamically
    bool symmetric_factors = false;                 // Create redundant factor pairs between robots

    // ID generators
    int next_rid_ = 0;                              // Next robot ID
    int next_vid_ = 0;                              // Next variable ID  
    int next_fid_ = 0;                              // Next factor ID
    int next_payload_id_ = 0;                       // Next payload ID

    // Entity containers
    std::map<int, std::shared_ptr<Robot>> robots_;  // All robots indexed by rid
    std::map<int, std::shared_ptr<Payload>> payloads_; // All payloads indexed by pid

    //==========================================================================
    // GRAPHICS & VISUALIZATION
    //==========================================================================
    Graphics* graphics;                             // Main graphics system
    Image obstacleImg;                              // Obstacle environment image

    //==========================================================================
    // SPATIAL INDEXING (KD-TREE)
    //==========================================================================
    typedef KDTreeMapOfVectorsAdaptor<std::map<int,std::vector<double>>> KDTree;
    std::map<int, std::vector<double>> robot_positions_{{0,{0.,0.}}};
    KDTree* treeOfRobots_;                         // Fast neighbor lookup

    //==========================================================================
    // PHYSICS ENGINES
    //==========================================================================
    // MuJoCo Physics & Rendering
    mjModel* mujoco_model_ = nullptr;
    mjData* mujoco_data_ = nullptr;

    mjvCamera mujoco_cam_;
    mjvOption mujoco_opt_;
    mjvScene mujoco_scn_;
    mjrContext mujoco_con_;
    GLFWwindow* mujoco_window_ = nullptr;

    std::map<int, int> robot_to_mujoco_id_;     // robot_id -> mujoco_body_id
    std::map<int, int> payload_to_mujoco_id_;   // payload_id -> mujoco_body_id
    std::map<int, int> mujoco_to_robot_id_;     // mujoco_body_id -> robot_id  
    std::map<int, int> mujoco_to_payload_id_;   // mujoco_body_id -> payload_id

    bool initializeMuJoCo();                   // 初始化MuJoCo世界
    void destroyMuJoCo();                      // 销毁MuJoCo世界
    bool loadMuJoCoModel(const std::string& xml_content);
    std::string generateMuJoCoXML();           // 动态生成XML模型

    // MuJoCo仿真控制
    void stepMuJoCo();                         // MuJoCo物理步进
    void resetMuJoCo();                        // 重置MuJoCo仿真

    // MuJoCo可视化
    bool initializeMuJoCoVisualization();
    void renderMuJoCo();
    void updateMuJoCoCamera();

    // 机器人创建（需要重新编译MuJoCo模型）
    int addRobotToModel(const Eigen::Vector2d& position, double radius, const std::string& name);
    int addPayloadToModel(const Eigen::Vector2d& position, double width, double height, const std::string& name);
    
    // 模型重建
    bool rebuildMuJoCoModel();                 // 当添加新实体时重建模型
    void updateEntityReferences();            // 更新实体的MuJoCo引用
    
    // 替换Box2D的碰撞检测
    bool areInContactMuJoCo(int body1_id, int body2_id) const;
    
    // 更新现有方法签名（保持接口，改为内部调用MuJoCo）
    bool isRobotContactingPayload(int robot_id, int payload_id);  // 内部调用areInContactMuJoCo
    std::pair<Eigen::Vector2d, Eigen::Vector2d> getContactPoint(int robot_id, int payload_id); // 内部调用getContactInfoMuJoCo
    std::pair<Eigen::Vector2d, Eigen::Vector2d> getContactInfoMuJoCo(int body1_id, int body2_id) const;

    void applyForcesToPayloadMuJoCo(std::shared_ptr<Payload> payload, 
                                   const Eigen::VectorXd& forces,
                                   const std::vector<Eigen::Vector2d>& contact_points,
                                   const std::vector<Eigen::Vector2d>& contact_normals);



    //==========================================================================
    // ROBOT & PAYLOAD MANAGEMENT
    //==========================================================================
    void createOrDeleteRobots();
    void deleteRobot(std::shared_ptr<Robot> robot);
    
    void createPayload(Eigen::Vector2d position, float width, float height);
    void deletePayload(int payload_id);
    std::shared_ptr<Payload> getPayload(int payload_id);

    //==========================================================================
    // CONTACT & COLLISION DETECTION
    //==========================================================================
    
    std::vector<Eigen::Vector2d> getFixedContactPoints(std::shared_ptr<Payload> payload);
    std::vector<Eigen::Vector2d> getFixedContactNormals(std::shared_ptr<Payload> payload);
    
    void assignContactPoints();
    std::vector<int> getNearbyRobots(int payload_id, double radius);
    void reassignLostContacts(int payload_id);

    //==========================================================================
    // PAYLOAD CONTROL & FORCE ALLOCATION
    //==========================================================================
    // Force allocation methods
    void computeLeastSquares();
    Eigen::VectorXd solveConstrainedLeastSquares(const Eigen::MatrixXd& G, const Eigen::Vector3d& w_cmd);
    void applyForcesToPayload(std::shared_ptr<Payload> payload, 
                             const Eigen::VectorXd& forces,
                             const std::vector<Eigen::Vector2d>& contact_points,
                             const std::vector<Eigen::Vector2d>& contact_normals);
    
    // Factor-based control
    void createForceAllocationFactors();
    void updateForceAllocationSystem();
    void updateDistributedPayloadControl();
    
    // Direct velocity control
    void applyDirectPayloadVelocityControl();
    Eigen::Vector2d computeDesiredPayloadVelocity(std::shared_ptr<Payload> payload);
    double computeDesiredPayloadAngularVelocity(std::shared_ptr<Payload> payload);
    
    // Wrench computation
    Eigen::Vector3d computeDesiredPayloadWrench(std::shared_ptr<Payload> payload);

    //==========================================================================
    // SIMULATION STEPPING & CONTROL
    //==========================================================================
    void timestep();                                // Main simulation step
    void moveRobot();                               // Legacy robot movement

    //==========================================================================
    // COMMUNICATION & NETWORKING
    //==========================================================================
    void calculateRobotNeighbours(std::map<int,std::shared_ptr<Robot>>& robots);
    void setCommsFailure(float failure_rate = globals.COMMS_FAILURE_RATE);

    //==========================================================================
    // GRAPHICS & RENDERING
    //==========================================================================
    void draw();                                    // Main rendering
    void draw_payloads();                          // Payload-specific rendering
    void eventHandler();                           // Input handling

    //==========================================================================
    // RANDOM NUMBER GENERATION
    //==========================================================================
private:
    std::mt19937 gen_normal = std::mt19937(globals.SEED);
    std::mt19937 gen_uniform = std::mt19937(globals.SEED);
    std::mt19937 gen_uniform_int = std::mt19937(globals.SEED);

    std::string xml_header_;
    std::string xml_robots_;
    std::string xml_payloads_;
    std::string xml_footer_;

    int next_mujoco_body_id_ = 0;
    bool model_needs_rebuild_ = false;

    // XML生成辅助
    std::string generateRobotXML(int robot_id, const Eigen::Vector2d& pos, double radius);
    std::string generatePayloadXML(int payload_id, const Eigen::Vector2d& pos, double width, double height);
    
    // ID管理
    int allocateMuJoCoBodyId();
    void deallocateMuJoCoBodyId(int body_id);
    
    // 错误处理
    void handleMuJoCoError(const std::string& operation);

public:
    template<typename T>
    T random_number(std::string distribution, T param1, T param2) {
        if (distribution == "normal") 
            return std::normal_distribution<T>(param1, param2)(gen_normal);
        if (distribution == "uniform") 
            return std::uniform_real_distribution<T>(param1, param2)(gen_uniform);
        return static_cast<T>(0);
    }
    
    int random_int(int lower, int upper) {
        return std::uniform_int_distribution<int>(lower, upper)(gen_uniform_int);
    }

    //==========================================================================
    // DEPRECATED/LEGACY METHODS
    //==========================================================================
    void isRobotContactngPayload(int robot_id, int payload_id);  // Typo in original - keep for compatibility
};