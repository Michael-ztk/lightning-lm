//
// Created by xiang on 25-11-13.
//

#ifndef LIGHTNING_EDGE_SE3_HEIGHT_PRIOR_H
#define LIGHTNING_EDGE_SE3_HEIGHT_PRIOR_H

#include "core/common/math.h"
#include "core/graph/base_unary_edge.h"
#include "core/types/vertex_se3.h"

namespace lightning::miao {

/**
 * 高度约束
 */
class EdgeHeightPrior : public BaseUnaryEdge<1, double, VertexSE3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void SetGravityDirection(const Vec3d& gravity_dir) { gravity_dir_ = gravity_dir; }

    void ComputeError() override {
        SE3 pose = ((VertexSE3 *)(vertices_[0]))->Estimate();
        error_[0] = gravity_dir_.dot(pose.translation()) - measurement_;
    }

    void LinearizeOplus() override {
        jacobian_oplus_xi_.setZero();
        jacobian_oplus_xi_.block<1, 3>(0, 0) = gravity_dir_.transpose();
    }

   private:
    Vec3d gravity_dir_ = Vec3d::UnitZ();
};
}  // namespace lightning::miao

#endif  // LIGHTNING_EDGE_SE3_HEIGHT_PRIOR_H
