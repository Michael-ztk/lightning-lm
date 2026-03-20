//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_KEYFRAME_H
#define LIGHTNING_KEYFRAME_H

#include "common/eigen_types.h"
#include "common/nav_state.h"
#include "common/point_def.h"
#include "common/std_types.h"

namespace lightning {

/// 关键帧描述
/// NOTE: 在添加后端后，需要加锁
class Keyframe {
   public:
    using Ptr = std::shared_ptr<Keyframe>;

    Keyframe() {}
    Keyframe(unsigned long id, CloudPtr cloud, NavState state)
        : id_(id), cloud_(cloud), state_(state), pose_lio_body_(state.GetPose()) {
        timestamp_ = state_.timestamp_;
        pose_opt_body_ = pose_lio_body_;
    }

    unsigned long GetID() const { return id_; }
    CloudPtr GetCloud() const { return cloud_; }

    SE3 GetLIOBodyPose() {
        UL lock(data_mutex_);
        return pose_lio_body_;
    }

    SE3 GetLIOLidarPose() {
        UL lock(data_mutex_);
        return pose_lio_body_ * GetBodyToLidarLocked();
    }

    void SetLIOBodyPose(const SE3& pose) {
        UL lock(data_mutex_);
        pose_lio_body_ = pose;
    }

    void SetLIOLidarPose(const SE3& pose) {
        UL lock(data_mutex_);
        pose_lio_body_ = pose * GetLidarToBodyLocked();
    }

    SE3 GetOptBodyPose() {
        UL lock(data_mutex_);
        return pose_opt_body_;
    }

    SE3 GetOptLidarPose() {
        UL lock(data_mutex_);
        return pose_opt_body_ * GetBodyToLidarLocked();
    }

    void SetOptBodyPose(const SE3& pose) {
        UL lock(data_mutex_);
        pose_opt_body_ = pose;
    }

    void SetOptLidarPose(const SE3& pose) {
        UL lock(data_mutex_);
        pose_opt_body_ = pose * GetLidarToBodyLocked();
    }

    void SetState(NavState s) {
        UL lock(data_mutex_);
        state_ = s;
    }

    NavState GetState() {
        UL lock(data_mutex_);
        return state_;
    }

   protected:
    unsigned long id_ = 0;

    double timestamp_ = 0;
    CloudPtr cloud_ = nullptr;  /// 降采样之后的点云

    std::mutex data_mutex_;
    SE3 pose_lio_body_;
    SE3 pose_opt_body_;

    NavState state_;  // 卡尔曼滤波器状态

   private:
    SE3 GetBodyToLidarLocked() const { return SE3(state_.offset_R_lidar_, state_.offset_t_lidar_); }
    SE3 GetLidarToBodyLocked() const { return GetBodyToLidarLocked().inverse(); }
};

}  // namespace lightning

#endif  // LIGHTNING_KEYFRAME_H
