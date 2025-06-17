/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <Utils.h>
#include <gbp/GBPCore.h>
#include <gbp/Factor.h>
#include <gbp/Variable.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <raylib.h>

/*****************************************************************************************************/
// Factor constructor
// Inputs:
//  - factor id (taken from simulator->next_f_id_++)
//  - robot id that this factor belongs to.
//  - A vector of pointers to Variables that the factor is to be connected to. Note, the order of the variables matters.
//  - sigma: factor strength. The factor precision Lambda = sigma^-2 * Identity
//  - measurement z: Eigen::VectorXd, must be same size as the output of the measurement function h().
//  - n_dofs is the number of degrees of freedom of the variables this factor is connected to. (eg. 4 for [x,y,xdot,ydot])
/*****************************************************************************************************/
Factor::Factor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
        float sigma, const Eigen::VectorXd& measurement, 
        int n_dofs) 
        : f_id_(f_id), r_id_(r_id), key_(r_id, f_id), variables_(variables), z_(measurement), n_dofs_(n_dofs) {

        // Initialise precision of the measurement function
        this->meas_model_lambda_ = Eigen::MatrixXd::Identity(z_.rows(), z_.rows()) / pow(sigma,2.);
        
        // Initialise empty inbox and outbox
        int n_dofs_total = 0; int n_dofs_var;
        for (auto var : variables_) {
            n_dofs_var = var->n_dofs_;
            Message zero_msg(n_dofs_var);
            inbox_[var->key_] = zero_msg;
            outbox_[var->key_] = zero_msg;
            n_dofs_total += n_dofs_var;
        }

        // This parameter useful if the factor is connected to another robot
        other_rid_=r_id_;                           

        // Initialise empty linearisation point
        X_ = Eigen::VectorXd::Zero(n_dofs_total);
    };

/*****************************************************************************************************/
// Destructor
/*****************************************************************************************************/
Factor::~Factor(){
}

/*****************************************************************************************************/
// Drawing function for the factor. Draws a 3d Cylinder (line-ish) between its connected variables
/*****************************************************************************************************/
void Factor::draw(){
    if ((factor_type_==DYNAMICS_FACTOR && globals.DRAW_PATH)){
        auto v_0 = variables_[0];
        auto v_1 = variables_[1];
        if (!v_0->valid_ || !v_1->valid_) {return;};
        DrawCylinderEx(Vector3{(float)v_0->mu_(0), globals.ROBOT_RADIUS, (float)v_0->mu_(1)},
                        Vector3{(float)v_1->mu_(0), globals.ROBOT_RADIUS, (float)v_1->mu_(1)}, 
                        0.1, 0.1, 4, BLACK);        
    }    
}

/*****************************************************************************************************/
// Default measurement function h_func_() is the identity function: it returns the variable.
/*****************************************************************************************************/
Eigen::MatrixXd h_func_(const Eigen::VectorXd& X){return X;};
/*****************************************************************************************************/
// Default measurement function Jacobian J_func_() is the first order taylor series jacobian by default.
// When defining new factors, custom h_func_() and J_func_() must be defined, otherwise defaults are used.
/*****************************************************************************************************/
Eigen::MatrixXd Factor::J_func_(const Eigen::VectorXd& X){return this->jacobianFirstOrder(X);};

Eigen::MatrixXd Factor::jacobianFirstOrder(const Eigen::VectorXd& X0){
    Eigen::MatrixXd h0 = h_func_(X0);    // Value at lin point
    Eigen::MatrixXd jac_out = Eigen::MatrixXd::Zero(h0.size(),X0.size());
    for (int i=0; i<X0.size(); i++){
        Eigen::VectorXd X_copy = X0;                                    // Copy of lin point
        X_copy(i) += delta_jac;                                         // Perturb by delta
        jac_out(Eigen::all, i) = (h_func_(X_copy) - h0) / delta_jac;    // Derivative (first order)
    }
    return jac_out;
};

/*****************************************************************************************************/
// Main section: Factor update:
// Messages from connected variables are aggregated. The beliefs are used to create the linearisation point X_.
// The Factor potential is calculated using h_func_ and J_func_
// The factor precision and information is created, and then marginalised to create outgoing messages to its connected variables.
/*****************************************************************************************************/
bool Factor::update_factor(){

    // Messages from connected variables are aggregated.
    // The beliefs are used to create the linearisation point X_.
    int idx = 0; int n_dofs;
    for (int v=0; v<variables_.size(); v++){
        n_dofs = variables_[v]->n_dofs_;
        auto& [_, __, mu_belief] = this->inbox_[variables_[v]->key_];
        X_(seqN(idx, n_dofs)) = mu_belief;
        idx += n_dofs;
    }

    // *Depending on the problem*, we may need to skip computation of this factor.
    // eg. to avoid extra computation, factor may not be required if two connected variables are too far apart.
    // in which case send out a Zero Message.
    if (this->skip_factor()){
        for (auto var : variables_){
            this->outbox_[var->key_] = Message(var->n_dofs_);
        }           
        return false;
    }
    
    // The Factor potential and linearised Factor Precision and Information is calculated using h_func_ and J_func_
    // residual() is by default (z - h_func_(X))
    // Skip calculation of Jacobian if the factor is linear and Jacobian has already been computed once
    h_ = h_func_(X_);
    J_ = (this->linear_ && this->initialised_)? J_ : this->J_func_(X_);
    Eigen::MatrixXd factor_lam_potential = J_.transpose() * meas_model_lambda_ * J_;
    Eigen::VectorXd factor_eta_potential = (J_.transpose() * meas_model_lambda_) * (J_ * X_ + residual());
    this->initialised_ = true;

    //  Update factor precision and information with incoming messages from connected variables.
    int marginalisation_idx = 0;
    for (int v_out_idx=0; v_out_idx<variables_.size(); v_out_idx++){
        auto var_out = variables_[v_out_idx];
        // Initialise with factor values
        Eigen::VectorXd factor_eta = factor_eta_potential;     
        Eigen::MatrixXd factor_lam = factor_lam_potential;
        
        // Combine the factor with the belief from other variables apart from the receiving variable
        int idx_v = 0;
        for (int v_idx=0; v_idx<variables_.size(); v_idx++){
            int n_dofs = variables_[v_idx]->n_dofs_;
            if (variables_[v_idx]->key_ != var_out->key_) {
                auto [eta_belief, lam_belief, _] = inbox_[variables_[v_idx]->key_];
                factor_eta(seqN(idx_v, n_dofs)) += eta_belief;
                factor_lam(seqN(idx_v, n_dofs), seqN(idx_v, n_dofs)) += lam_belief;
            }
            idx_v += n_dofs;
        }
        
        // Marginalise the Factor Precision and Information to send to the relevant variable
        outbox_[var_out->key_] = marginalise_factor_dist(factor_eta, factor_lam, v_out_idx, marginalisation_idx);
        marginalisation_idx += var_out->n_dofs_;
    }

    return true;
};

/*****************************************************************************************************/
// Marginalise the factor Precision and Information and create the outgoing message to the variable
/*****************************************************************************************************/
Message Factor::marginalise_factor_dist(const Eigen::VectorXd &eta, const Eigen::MatrixXd &Lam, int var_idx, int marg_idx){
    // Marginalisation only needed if factor is connected to >1 variables
    int n_dofs = variables_[var_idx]->n_dofs_;
    if (eta.size() == n_dofs) return Message {eta, Lam};

    Eigen::VectorXd eta_a(n_dofs), eta_b(eta.size()-n_dofs);
    eta_a = eta(seqN(marg_idx, n_dofs));
    eta_b << eta(seq(0, marg_idx - 1)), eta(seq(marg_idx + n_dofs, last));

    Eigen::MatrixXd lam_aa(n_dofs, n_dofs), lam_ab(n_dofs, Lam.cols()-n_dofs);
    Eigen::MatrixXd lam_ba(Lam.rows()-n_dofs, n_dofs), lam_bb(Lam.rows()-n_dofs, Lam.cols()-n_dofs);
    lam_aa << Lam(seqN(marg_idx, n_dofs), seqN(marg_idx, n_dofs));
    lam_ab << Lam(seqN(marg_idx, n_dofs), seq(0, marg_idx - 1)), Lam(seqN(marg_idx, n_dofs), seq(marg_idx + n_dofs, last));
    lam_ba << Lam(seq(0, marg_idx - 1), seq(marg_idx, marg_idx + n_dofs - 1)), Lam(seq(marg_idx + n_dofs, last), seqN(marg_idx, n_dofs));
    lam_bb << Lam(seq(0, marg_idx - 1), seq(0, marg_idx - 1)), Lam(seq(0, marg_idx - 1), seq(marg_idx + n_dofs, last)),
            Lam(seq(marg_idx + n_dofs, last), seq(0, marg_idx - 1)), Lam(seq(marg_idx + n_dofs, last), seq(marg_idx + n_dofs, last));

    Eigen::MatrixXd lam_bb_inv = lam_bb.inverse();
    Message marginalised_msg(n_dofs);
    marginalised_msg.eta = eta_a - lam_ab * lam_bb_inv * eta_b;
    marginalised_msg.lambda = lam_aa - lam_ab * lam_bb_inv * lam_ba;
    if (!marginalised_msg.lambda.allFinite()) marginalised_msg.setZero();

    return marginalised_msg;
};    

/********************************************************************************************/
/********************************************************************************************/
//                      CUSTOM FACTORS SPECIFIC TO THE PROBLEM
// Create a new factor definition as shown with these examples.
// You may create a new factor_type_, in the enum in Factor.h (optional, default type is DEFAULT_FACTOR)
// Create a measurement function h_func_() and optionally Jacobian J_func_().

// 在文件末尾添加ContactFactor实现
ContactFactor::ContactFactor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
                             float sigma, const Eigen::VectorXd& measurement,
                             std::shared_ptr<Payload> payload,
                             Eigen::Vector2d target_contact_point,
                             Simulator* sim)
    : Factor{f_id, r_id, variables, sigma, measurement},
      payload_(payload), target_contact_point_(target_contact_point), sim_(sim) {
    factor_type_ = CONTACT_FACTOR;
    this->delta_jac = 1e-3;
}

Eigen::MatrixXd ContactFactor::h_func_(const Eigen::VectorXd& X) {
    Eigen::Vector2d robot_pos = X.head(2);
    double contact_error = computeContactError(robot_pos);
    
    Eigen::MatrixXd h(1, 1);
    h(0, 0) = contact_error;
    return h;
}

double ContactFactor::computeContactError(const Eigen::Vector2d& robot_pos) {
    Eigen::Vector2d world_target_contact = payloadToWorld(target_contact_point_);
    double distance_to_target = (robot_pos - world_target_contact).norm();
    return distance_to_target - globals.ROBOT_RADIUS;
}

Eigen::Vector2d ContactFactor::payloadToWorld(const Eigen::Vector2d& local_point) {
    Eigen::Vector2d payload_pos = payload_->getPosition();
    double rotation = payload_->rotation_;
    
    Eigen::Matrix2d rot;
    rot << cos(rotation), -sin(rotation),
           sin(rotation),  cos(rotation);
    
    return payload_pos + rot * local_point;
}

bool ContactFactor::skip_factor() {
    return false; // 与dynamics factor生命周期一致
}

void ContactFactor::draw() {
    if (globals.DRAW_PATH) {
        Eigen::Vector2d world_target = payloadToWorld(target_contact_point_);
        Vector3 target_pos = {(float)world_target.x(), 1.0f, (float)world_target.y()};
        DrawSphere(target_pos, 0.3f, PURPLE);
        
        if (!variables_.empty() && variables_[0]->valid_) {
            Vector3 robot_pos = {(float)variables_[0]->mu_(0), 1.0f, (float)variables_[0]->mu_(1)};
            double error = computeContactError(variables_[0]->mu_.head(2));
            Color line_color = (abs(error) < 0.1) ? GREEN : RED;
            DrawLine3D(robot_pos, target_pos, line_color);
        }
    }
}

// PayloadVelocityFactor实现
PayloadVelocityFactor::PayloadVelocityFactor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
                                             float sigma, const Eigen::VectorXd& measurement,
                                             std::shared_ptr<Payload> payload,
                                             Eigen::Vector2d contact_normal)
    : Factor{f_id, r_id, variables, sigma, measurement},
      payload_(payload), contact_normal_(contact_normal) {
    factor_type_ = PAYLOAD_VELOCITY_FACTOR;
    this->delta_jac = 1e-3;
}

Eigen::MatrixXd PayloadVelocityFactor::h_func_(const Eigen::VectorXd& X) {
    Eigen::Vector2d robot_pos = X.head(2);
    Eigen::Vector2d robot_velocity = X.tail(2);
    
    auto [desired_linear_vel, desired_angular_vel] = computeDesiredPayloadMotion();
    auto [linear_contribution, angular_contribution] = computeRobotContribution(robot_pos, robot_velocity);
    
    Eigen::MatrixXd h(2, 1);
    h(0, 0) = linear_contribution - desired_linear_vel;
    h(1, 0) = angular_contribution - desired_angular_vel;
    
    return h;
}

// std::pair<double, double> PayloadVelocityFactor::computeDesiredPayloadMotion() {
//     if (!payload_) return {0.0, 0.0};
    
//     // 线速度分量
//     Eigen::Vector2d payload_to_target = payload_->target_position_ - payload_->position_;
//     double desired_linear_velocity_component = 0.0;
    
//     if (payload_to_target.norm() > 0.1) {
//         Eigen::Vector2d desired_velocity = payload_to_target.normalized() * globals.MAX_SPEED * 0.5;
//         desired_linear_velocity_component = desired_velocity.dot(contact_normal_);
//     }
    
//     // 角速度（如果有目标朝向）
//     double desired_angular_velocity = 0.0;
//     // 这里可以根据需要添加旋转控制逻辑
    
//     return {desired_linear_velocity_component, desired_angular_velocity};
// }

std::pair<double, double> PayloadVelocityFactor::computeDesiredPayloadMotion() {
    if (!payload_) return {0.0, 0.0};
    
    // 线速度分量（保持现有逻辑）
    Eigen::Vector2d payload_to_target = payload_->target_position_ - payload_->position_;
    double desired_linear_velocity_component = 0.0;
    
    if (payload_to_target.norm() > 0.1) {
        Eigen::Vector2d desired_velocity = payload_to_target.normalized() * globals.MAX_SPEED * 0.5;
        desired_linear_velocity_component = desired_velocity.dot(contact_normal_);
    }
    
    // 角速度（使用修复后的旋转误差计算）
    double desired_angular_velocity = 0.0;
    double rotation_error = payload_->getRotationError();
    
    if (std::abs(rotation_error) > 0.01) {  // 只有当旋转误差显著时才施加角速度
        desired_angular_velocity = std::copysign(1.0, rotation_error) * 
            std::min(static_cast<double>(globals.MAX_ANGULAR_SPEED * 0.5), std::abs(rotation_error));
        
        // 调试输出
        static int debug_counter = 0;
        if (debug_counter++ % 60 == 0) {
            std::cout << "Rotation error: " << rotation_error << " rad, desired angular vel: " 
                      << desired_angular_velocity << std::endl;
        }
    }
    
    return {desired_linear_velocity_component, desired_angular_velocity};
}

std::pair<double, double> PayloadVelocityFactor::computeRobotContribution(
    const Eigen::Vector2d& robot_pos, const Eigen::Vector2d& robot_velocity) {
    
    double linear_contribution = robot_velocity.dot(contact_normal_);
    double angular_contribution = 0.0;
    
    if (payload_) {
        Eigen::Vector2d payload_center = payload_->getPosition();
        Eigen::Vector2d contact_point = robot_pos - globals.ROBOT_RADIUS * contact_normal_;
        Eigen::Vector2d r = contact_point - payload_center;
        
        double torque_arm = std::abs(r.x() * contact_normal_.y() - r.y() * contact_normal_.x());
        angular_contribution = torque_arm * linear_contribution;
        
        double payload_inertia_approx = payload_->getMomentOfInertia();
        if (payload_inertia_approx > 1e-6) {
            angular_contribution /= payload_inertia_approx;
        }
    }
    
    return {linear_contribution, angular_contribution};
}

bool PayloadVelocityFactor::skip_factor() {
    if (payload_) {
        double distance_to_target = (payload_->target_position_ - payload_->position_).norm();
        return distance_to_target < 0.5;
    }
    return false;
}

/********************************************************************************************/
/* Dynamics factor: constant-velocity model */
/*****************************************************************************************************/
DynamicsFactor::DynamicsFactor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
    float sigma, const Eigen::VectorXd& measurement, 
    float dt)
    : Factor{f_id, r_id, variables, sigma, measurement}{ 
        factor_type_ = DYNAMICS_FACTOR;
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_dofs_/2,n_dofs_/2);
        Eigen::MatrixXd O = Eigen::MatrixXd::Zero(n_dofs_/2,n_dofs_/2);
        Eigen::MatrixXd Qc_inv = pow(sigma, -2.) * I;

        Eigen::MatrixXd Qi_inv(n_dofs_, n_dofs_);
        Qi_inv << 12.*pow(dt, -3.) * Qc_inv,   -6.*pow(dt, -2.) * Qc_inv,
                  -6.*pow(dt, -2.) * Qc_inv,   4./dt * Qc_inv;   

        this->meas_model_lambda_ = Qi_inv;        

        // Store Jacobian as it is linear
        this->linear_ = true;
        J_ = Eigen::MatrixXd::Zero(n_dofs_, n_dofs_*2);
        J_ << I, dt*I, -1*I,    O,
             O,    I,    O, -1*I; 

    };

Eigen::MatrixXd DynamicsFactor::h_func_(const Eigen::VectorXd& X){
    return J_ * X;
}    
Eigen::MatrixXd DynamicsFactor::J_func_(const Eigen::VectorXd& X){
    return J_;
}

/********************************************************************************************/
/* Interrobot factor: for avoidance of other robots */
// This factor results in a high energy or cost if two robots are planning to be in the same 
// position at the same timestep (collision). This factor is created between variables of two robots.
// The factor has 0 energy if the variables are further away than the safety distance. skip_ = true in this case.
/********************************************************************************************/
// PayloadFactor::PayloadFactor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
//     float sigma, const Eigen::VectorXd& measurement,
//     float robot_radius)
//     : Factor{f_id, r_id, variables, sigma, measurement} {
//         factor_type_ = PAYLOAD_FACTOR;
//         // Change the parameters below:
//         float eps = 0.2 * robot_radius;
//         this->safety_distance_ = 2*robot_radius + eps; // Safety distance between two robots
//         this->delta_jac = 1e-2;
//     };

InterrobotFactor::InterrobotFactor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
    float sigma, const Eigen::VectorXd& measurement, 
    float robot_radius)
    : Factor{f_id, r_id, variables, sigma, measurement} {  
        factor_type_ = INTERROBOT_FACTOR;
        float eps = 0.2 * robot_radius;
        this->safety_distance_ = 2*robot_radius + eps;
        this->delta_jac = 1e-2;
};

Eigen::MatrixXd InterrobotFactor::h_func_(const Eigen::VectorXd& X){
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(z_.rows(),z_.cols());
    Eigen::VectorXd X_diff = X(seqN(0,n_dofs_/2)) - X(seqN(n_dofs_, n_dofs_/2));
    X_diff += 1e-6*r_id_*Eigen::VectorXd::Ones(n_dofs_/2);

    double r = X_diff.norm();
    if (r <= safety_distance_){
        this->skip_flag = false;
        h(0) = 1.f*(1 - r/safety_distance_);
    }
    else {
        this->skip_flag = true;
    }

    return h;
};

Eigen::MatrixXd InterrobotFactor::J_func_(const Eigen::VectorXd& X){
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(z_.rows(), n_dofs_*2);
    Eigen::VectorXd X_diff = X(seqN(0,n_dofs_/2)) - X(seqN(n_dofs_, n_dofs_/2));
    X_diff += 1e-6*r_id_*Eigen::VectorXd::Ones(n_dofs_/2);// Add a tiny random offset to avoid div/0 errors
    double r = X_diff.norm();
    if (r <= safety_distance_){
        J(0,seqN(0, n_dofs_/2)) = -1.f/safety_distance_/r * X_diff;
        J(0,seqN(n_dofs_, n_dofs_/2)) = 1.f/safety_distance_/r * X_diff;
    }
    return J;
};

bool InterrobotFactor::skip_factor(){
    this->skip_flag = ( (X_(seqN(0,n_dofs_/2)) - X_(seqN(n_dofs_, n_dofs_/2))).squaredNorm() >= safety_distance_*safety_distance_ );
    return this->skip_flag;
}


/********************************************************************************************/
// Obstacle factor for static obstacles in the scene. This factor takes a pointer to the obstacle image from the Simulator.
// Note. in the obstacle image, white areas represent obstacles (as they have a value of 1).
// The input image to the simulator is opposite, which is why it needs to be inverted.
// The delta used in the first order jacobian calculation is chosen such that it represents one pixel in the image.
/********************************************************************************************/
ObstacleFactor::ObstacleFactor(Simulator* sim, int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
    float sigma, const Eigen::VectorXd& measurement, Image* p_obstacleImage)
    : Factor{f_id, r_id, variables, sigma, measurement}, p_obstacleImage_(p_obstacleImage){
        factor_type_ = OBSTACLE_FACTOR;
        this->delta_jac = 1.*(float)globals.WORLD_SZ / (float)p_obstacleImage->width;
};
Eigen::MatrixXd ObstacleFactor::h_func_(const Eigen::VectorXd& X){
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(1,1);
    // White areas are obstacles, so h(0) should return a 1 for these regions.
    float scale = p_obstacleImage_->width / (float)globals.WORLD_SZ;
    Vector3 c_hsv = ColorToHSV(GetImageColor(*p_obstacleImage_, (int)((X(0) + globals.WORLD_SZ/2) * scale), (int)((X(1) + globals.WORLD_SZ/2) * scale)));
    h(0) = c_hsv.z;
    return h;
};