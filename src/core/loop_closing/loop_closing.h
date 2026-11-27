//
// Created by xiang on 25-4-21.
//

#ifndef LIGHTNING_LOOP_CLOSING_H
#define LIGHTNING_LOOP_CLOSING_H

#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "utils/async_message_process.h"

#include "core/graph/optimizer.h"
#include "core/types/edge_se3.h"

namespace lightning {

/**
 * 基于grid ndt的回环检测
 */
class LoopClosing {
   public:
    struct Options {
        Options() {}

        bool verbose_ = true;       // 输出调试信息
        bool online_mode_ = false;  // 切换离线-在线模式

        int loop_kf_gap_ = 20;       // 每隔多少个关键帧检查一次
        int min_id_interval_ = 20;   // 被检查的关键帧ID间隔
        int closest_id_th_ = 50;     // 历史关键帧与当前帧的ID间隔
        double max_range_ = 30.0;    // 候选帧的最大距离
        double ndt_score_th_ = 1.0;  // ndt位姿分值
        int source_submap_idx_range_ = 0;
        int source_submap_stride_ = 1;

        /// 图优化权重
        double motion_trans_noise_ = 0.1;               // 位移权重
        double motion_rot_noise_ = 3.0 * M_PI / 180.0;  // 旋转权重

        double loop_trans_noise_ = 0.2;               // 位移权重
        double loop_rot_noise_ = 3.0 * M_PI / 180.0;  // 旋转权重

        double rk_loop_th_ = 5.2 / 5;  // 回环的RK阈值

        bool with_height_ = true;
        double height_noise_ = 0.1;
        bool save_loop_clouds_ = false;  // 保存回环点云用于调试
        bool remove_ground_in_lc_ = false;  // 子图构建时是否移除地面

        // 子图几何一致性检查（配准前）
        double max_submap_aspect_ = 5.0;      // 子图 XY 长宽比上限（0=禁用）
        double max_centroid_dist_ = 2.0;      // 子图质心距上限(m)（0=禁用）
        double min_coverage_rate_ = 75.0;     // 质心超限时的最小覆盖度(%)，取 max(正向,反向)
        double coverage_cell_size_ = 0.2;     // 计算覆盖度时的 XY 栅格边长(m)

        // GICP 主配准参数（与 rk100 一致）
        double gicp_reg_voxel_ = 0.15;      // 降采样体素(m)
        int gicp_reg_max_iter_ = 60;        // 最大迭代次数
        double gicp_reg_corr_dist_ = 1.0;   // 最大对应点距离(m)

        // 空间去重：同一区域在 半径+时间+帧数 三个窗口内只触发一次
        double suppress_radius_ = 3.0;   // 空间半径(m)，0=禁用
        double suppress_time_ = 300.0;   // 时间窗口(s)
        int suppress_kf_ = 600;          // 帧数窗口

        /// 墙面中位数精修（GICP 之后）
        /// GICP 最小化全局 RMSE（平方和），对离群点敏感；地面点在任何水平平移下
        /// 最近邻距离几乎不变（平坦、无约束），墙面才是走廊轴向位移的唯一约束来源。
        /// 因此走廊场景容易收敛到"墙错开十几厘米"的解。改用墙面残差【中位数】
        /// （抗离群）直接优化 (dx,dy,dyaw) 可修正这类解，且在已正确的解上几乎不动。
        bool enable_wall_refine_ = true;
        double wall_refine_thresh_ = 0.10;    // 精修后墙面残差中位数上限(m)，超过则拒绝
        double wall_refine_max_move_ = 0.5;   // 允许的最大修正量(m)
        double wall_normal_z_thr_ = 0.3;      // 竖直面判定：|nz| < 该值

        /// 配准模式：
        ///   "gicp_wall"（默认，incumbent）：GICP 主配准 + 墙面中位数精修
        ///   "thinwall"（challenger）：单遍紧核薄墙配准，从轨迹初值出发整体替代 GICP+精修
        ///   （rk100 上验证过的 full_regress.py 算法的 C++ 移植：紧 Huber 核 + 平面性/距离加权
        ///    + 残差门限调度 + 定向法向门控；修正量限定平面自由度 (dx,dy,dyaw)，z/roll/pitch 锁定在初值）
        std::string reg_mode_ = "thinwall";

        // ---- thinwall 配准参数（challenger）----
        int thinwall_max_outer_ = 10;        // IRLS 外层迭代数（对应验证脚本的 iters）
        double thinwall_huber_delta_ = 0.05; // 紧 Huber 核阈值 FSCALE（紧核是精度关键）
        int thinwall_knn_ = 20;              // 目标邻域 KNN 数（邻域平面拟合用）
        double thinwall_nn_radius_ = 1.2;    // 目标邻域最大半径(m)
        double thinwall_floor_z_ = -1.0;     // 地面判定：z < 该值 → 法向强制 +z
        double thinwall_ceil_z_ = 1.8;       // 天花板判定：z > 该值 → 法向强制 -z
    };

    LoopClosing(Options options = Options()) { options_ = options; }
    ~LoopClosing();

    void Init(const std::string yaml_path);

    /// 向回环中添加一个关键帧
    void AddKF(Keyframe::Ptr kf);

    /// 如果检测到新地回环并发生了优化，则调用回调
    using LoopClosedCallback = std::function<void()>;
    void SetLoopClosedCB(LoopClosedCallback cb) { loop_cb_ = cb; }

   protected:
    void HandleKF(Keyframe::Ptr kf);

    void DetectLoopCandidates();

    /// 计算回环候选位姿
    void ComputeLoopCandidates();

    /// 计算单个回环候选
    void ComputeForCandidate(LoopCandidate& c);

    /// 墙面中位数精修：在 GICP 结果基础上优化 (dx,dy,dyaw)，使墙面残差中位数最小
    /// @param source    源点云（已按 GICP 结果变换到目标坐标系）
    /// @param target    目标子图
    /// @param T         输入输出：待精修的位姿
    /// @param residual  输出：精修后的墙面残差中位数(m)
    /// @return 残差是否低于阈值（true=接受）
    bool RefineByWallMedian(const CloudPtr& source, const CloudPtr& target, SE3& T, double& residual);

    /// 单遍紧核薄墙配准（challenger，reg_mode = "thinwall"）
    /// 从轨迹初值 T_init 出发的 IRLS 点到面配准：定向法向门控 + 邻域平面拟合 +
    /// 平面性/距离加权 + 残差门限调度 + 紧 Huber 核 GN。修正量限定平面自由度
    /// (dx,dy,dyaw)（绕源质心旋转），z/roll/pitch 锁定在初值。
    /// @param source  源点云（源帧局部系）
    /// @param target  目标点云（世界系）
    /// @param T_init  轨迹初值（世界 <- 源局部）
    /// @param T_final 输出：配准结果（= Delta(dx,dy,dyaw) * T_init）
    /// @param move    输出：修正量 |(dx,dy)|(m)，供 move 上限验收
    /// @return 配准流程是否完成（验收由调用方以统一尺子度量）
    bool RegisterByThinWall(const CloudPtr& source, const CloudPtr& target, const SE3& T_init, SE3& T_final,
                            double& move);

    /// 统一验收尺子：cloud（目标系）到全分辨率 target 的墙面 NN 残差中位数
    /// （与 incumbent RefineByWallMedian 内部完全相同的度量）
    double WallResidualOnFullTarget(const CloudPtr& cloud, const CloudPtr& target_full);

    /// 优化位姿
    void PoseOptimization();

    Options options_;

    Keyframe::Ptr last_kf_ = nullptr;
    Keyframe::Ptr last_loop_kf_ = nullptr;
    Keyframe::Ptr cur_kf_ = nullptr;
    std::vector<Keyframe::Ptr> all_keyframes_;
    std::vector<LoopCandidate> candidates_;

    /// 已触发过回环的区域（用于空间去重）
    struct LoopRegion {
        Vec3d center;
        double stamp = 0.0;
        int kf_id = 0;
    };
    std::vector<LoopRegion> recent_loop_regions_;

    AsyncMessageProcess<Keyframe::Ptr> kf_thread_;

    std::shared_ptr<miao::Optimizer> optimizer_ = nullptr;

    Mat6d info_motion_ = Mat6d::Identity();  // 关键帧间的运动信息阵
    Mat6d info_loops_ = Mat6d::Identity();   // 回环帧的信息矩阵

    std::vector<std::shared_ptr<miao::VertexSE3>> kf_vert_;
    std::vector<std::shared_ptr<miao::EdgeSE3>> edge_loops_;

    LoopClosedCallback loop_cb_;
};

}  // namespace lightning

#endif  // LIGHTNING_LOOP_CLOSING_H
