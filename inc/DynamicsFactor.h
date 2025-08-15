#ifndef DYNAMICS_FACTOR_H
#define DYNAMICS_FACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Key.h>
#include <gtsam/base/Matrix.h>

class DynamicsFactor : public gtsam::NoiseModelFactor2<gtsam::Vector4, gtsam::Vector4> {
private:
    typedef gtsam::NoiseModelFactor2<gtsam::Vector4, gtsam::Vector4> Base;
    double dt_;

public:
    // Provide access to the Matrix& version of evaluateError:
    using gtsam::NoiseModelFactor2<gtsam::Vector4, gtsam::Vector4>::evaluateError;

    DynamicsFactor(gtsam::Key key1, gtsam::Key key2, double dt, 
                   const gtsam::SharedNoiseModel& model)
        : Base(model, key1, key2), dt_(dt) {}

    virtual ~DynamicsFactor() {}

    gtsam::Vector evaluateError(const gtsam::Vector4& x1, const gtsam::Vector4& x2,
                               gtsam::OptionalMatrixType H1,
                               gtsam::OptionalMatrixType H2) const override;

    virtual gtsam::NonlinearFactor::shared_ptr clone() const override {
        return std::make_shared<DynamicsFactor>(*this);
    }
};

#endif // DYNAMICS_FACTOR_H