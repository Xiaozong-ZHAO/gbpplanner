/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <Eigen/Core>
#include <Eigen/Dense>
#include <raylib.h>
#include <vector>
#include <math.h>
#include <Utils.h>
#include <gbp/GBPCore.h>

#include <gbp/Variable.h>
#include <gbp/Factor.h>

/***********************************************************************************************************/
// Variable constructor
// Inputs:
// - variable id (vid, taken from simulator->next_v_id_++)
// - id of the robot this variable belongs to
// - mu_prior: mean vector (mu) of the prior on the variable
// - sigma_prior_list: an Eigen::VectorXd of length n_dofs representing the strength of the prior
//      (eg. sigma_prior_list(0)^2 is the covariance of the first element of the variable (x coordinate).
//      This is used to create the Precision (Lambda) of the prior as Lambda = diag(sigma_prior_list)
/***********************************************************************************************************/
// Variable::Variable(int v_id, int r_id, const Eigen::VectorXd& mu_prior, const Eigen::VectorXd& sigma_prior_list,
//                    float size, int n_dofs)
//             : v_id_(v_id), r_id_(r_id), key_(Key(r_id, v_id)), mu_(mu_prior), size_(size), n_dofs_(n_dofs) {
//                 lam_prior_ = sigma_prior_list.cwiseProduct(sigma_prior_list).cwiseInverse().asDiagonal();
//                 if (!lam_prior_.allFinite()) lam_prior_.setZero();
//                 eta_prior_ = lam_prior_ * mu_prior;
//                 belief_ = Message(eta_prior_, lam_prior_, mu_prior);

//             };

// Variable::Variable(int v_id, int r_id, const Eigen::VectorXd& mu_prior, const Eigen::VectorXd& sigma_prior_list,
//                    float size, int n_dofs)
//             : v_id_(v_id), r_id_(r_id), key_(Key(r_id, v_id)), mu_(mu_prior), size_(size), n_dofs_(n_dofs) {
                
//     lam_prior_ = sigma_prior_list.cwiseProduct(sigma_prior_list).cwiseInverse().asDiagonal();
//     if (!lam_prior_.allFinite()) lam_prior_.setZero();
//     eta_prior_ = lam_prior_ * mu_prior;
    
//     // *** 问题可能在这里 ***
//     // 原来的代码可能是：
//     // belief_ = Message(eta_prior_, lam_prior_, mu_prior);
    
//     // 修复：确保Message使用正确的维度
//     belief_ = Message(n_dofs);  // 先创建正确维度的空消息
//     belief_.eta = eta_prior_;   // 然后设置内容
//     belief_.lambda = lam_prior_;
//     belief_.mu = mu_prior;
// };

Variable::Variable(int v_id, int r_id, const Eigen::VectorXd& mu_prior, const Eigen::VectorXd& sigma_prior_list,
                   float size, int n_dofs)
            : v_id_(v_id), r_id_(r_id), key_(Key(r_id, v_id)), mu_(mu_prior), size_(size), n_dofs_(n_dofs) {
    
    // 计算精度矩阵，添加数值稳定性检查
    Eigen::VectorXd precision_diag = sigma_prior_list.cwiseProduct(sigma_prior_list).cwiseInverse();
    
    // *** 关键修复：避免无穷大精度 ***
    const double max_precision = 1e12;  // 限制最大精度
    const double min_precision = 1e-12; // 限制最小精度
    
    for (int i = 0; i < precision_diag.size(); i++) {
        if (!std::isfinite(precision_diag(i)) || precision_diag(i) > max_precision) {
            precision_diag(i) = max_precision;
            // std::cout << "Clamped precision for variable (" << r_id << "," << v_id 
            //           << ") element " << i << std::endl;
        } else if (precision_diag(i) < min_precision) {
            precision_diag(i) = min_precision;
        }
    }
    
    lam_prior_ = precision_diag.asDiagonal();
    eta_prior_ = lam_prior_ * mu_prior;
    
    // 确保belief正确初始化
    belief_ = Message(n_dofs);
    belief_.eta = eta_prior_;
    belief_.lambda = lam_prior_;
    belief_.mu = mu_prior;
    
    // 验证初始化
    if (!belief_.lambda.allFinite() || !belief_.mu.allFinite()) {
        std::cerr << "[ERROR] Variable (" << r_id << "," << v_id 
                  << ") initialized with invalid belief!" << std::endl;
        
        // 强制设置为有效状态
        belief_.lambda = Eigen::MatrixXd::Identity(n_dofs, n_dofs) * 1e-6;
        belief_.eta = belief_.lambda * mu_prior;
        belief_.mu = mu_prior;
    }
}

/***********************************************************************************************************/
// Destructor: deletes all connected factors too
/***********************************************************************************************************/
Variable::~Variable(){
    for (auto [fkey, fac] : factors_){
        delete_factor(fkey);
    }
}

/***********************************************************************************************************/
// Variable belief update step:
// Aggregates all the messages from its connected factors (begins with the prior, as this is effectively a unary factor)
// The valid_ flag is useful for drawing the variable.
// Finally the outgoing messages to factors is created.
/***********************************************************************************************************/
void Variable::update_belief(){
    // Collect messages from all other factors, begin by "collecting message from pose factor prior"
    eta_ = eta_prior_;
    lam_ = lam_prior_;

    for (auto& [f_key, msg] : inbox_) {
        auto [eta_msg, lam_msg, _] = msg;
        eta_ += eta_msg;
        lam_ += lam_msg;
    }
    
    // Update belief
    sigma_ = lam_.inverse();
    valid_ = sigma_.allFinite();
    if (valid_) mu_ = sigma_ * eta_;
    belief_ = Message {eta_, lam_, mu_};

    // Create message to send to each factor that sent it stuff
    // msg is the aggregate of all OTHER factor messages (belief - last sent msg of that factor)
    for (auto [f_key, fac] : factors_) {
        outbox_[f_key] = belief_ - inbox_.at(f_key);
    }
}

/***********************************************************************************************************/
// Changes the prior on the variable. It updates the belief_ of the variable.
/***********************************************************************************************************/
void Variable::change_variable_prior(const Eigen::VectorXd& new_mu){
    eta_prior_ = lam_prior_ * new_mu;
    mu_ = new_mu;
    belief_ = Message {eta_, lam_, mu_};
    for (auto [fkey, fac] : factors_){
        outbox_[fkey] = belief_;
        inbox_[fkey].setZero();
    }
};

/***********************************************************************************************************/
// Add a factor to this variable's list of factors, and initialise an outgoing message of its belief.
/***********************************************************************************************************/
void Variable::add_factor(std::shared_ptr<Factor> fac){
    factors_[fac->key_] = fac;
    inbox_[fac->key_] = Message(n_dofs_);
    outbox_[fac->key_] = belief_;
}

/***********************************************************************************************************/
// Delete a factor from this variable's list of factors. Remove it from its inbox too.
/***********************************************************************************************************/
void Variable::delete_factor(Key fac_key){
    factors_.erase(fac_key);
    inbox_.erase(fac_key);
}

/***********************************************************************************************************/
// Default drawing function for the variable. A custom function can be set by setting draw_fn_ = a lambda[](){} function
/***********************************************************************************************************/
void Variable::draw(bool filled){
    if (draw_fn_!=NULL){
        draw_fn_;
    } else {
        // Default variable draw function.
        if (!valid_) return;
        const int var_x = (mu_(0) );
        const int var_y = (mu_(1) );
        DrawSphere(Vector3{(float)var_x, size_, (float)var_y}, size_, BLACK);
    }
}