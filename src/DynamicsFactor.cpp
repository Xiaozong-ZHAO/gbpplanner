#include "DynamicsFactor.h"
#include <gtsam/base/Matrix.h>

gtsam::Vector DynamicsFactor::evaluateError(const gtsam::Vector4& x1, const gtsam::Vector4& x2,
                                           gtsam::OptionalMatrixType H1,
                                           gtsam::OptionalMatrixType H2) const {
    
    // x1 = [x_i, y_i, xdot_i, ydot_i]
    // x2 = [x_{i+1}, y_{i+1}, xdot_{i+1}, ydot_{i+1}]
    
    // Dynamics model: h(x_i, x_{i+1}) = 
    // [x_{i+1} - x_i - dt*xdot_i]
    // [y_{i+1} - y_i - dt*ydot_i] 
    // [xdot_{i+1} - xdot_i]
    // [ydot_{i+1} - ydot_i]
    
    gtsam::Vector4 error;
    error(0) = x2(0) - x1(0) - dt_ * x1(2);  // x_{i+1} - x_i - dt*xdot_i
    error(1) = x2(1) - x1(1) - dt_ * x1(3);  // y_{i+1} - y_i - dt*ydot_i
    error(2) = x2(2) - x1(2);                // xdot_{i+1} - xdot_i
    error(3) = x2(3) - x1(3);                // ydot_{i+1} - ydot_i
    
    // Compute Jacobians if requested
    if (H1) {
        *H1 = gtsam::Matrix::Zero(4, 4);
        (*H1)(0, 0) = -1.0;    // ∂e₀/∂x_i
        (*H1)(0, 2) = -dt_;    // ∂e₀/∂xdot_i
        (*H1)(1, 1) = -1.0;    // ∂e₁/∂y_i
        (*H1)(1, 3) = -dt_;    // ∂e₁/∂ydot_i
        (*H1)(2, 2) = -1.0;    // ∂e₂/∂xdot_i
        (*H1)(3, 3) = -1.0;    // ∂e₃/∂ydot_i
    }
    
    if (H2) {
        *H2 = gtsam::Matrix::Zero(4, 4);
        (*H2)(0, 0) = 1.0;     // ∂e₀/∂x_{i+1}
        (*H2)(1, 1) = 1.0;     // ∂e₁/∂y_{i+1}
        (*H2)(2, 2) = 1.0;     // ∂e₂/∂xdot_{i+1}
        (*H2)(3, 3) = 1.0;     // ∂e₃/∂ydot_{i+1}
    }
    
    return error;
}