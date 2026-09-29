//
// Created by xiang on 2022/2/15.
//

#include "core/lio/eskf.hpp"

#include <Eigen/Eigenvalues>

namespace lightning {

void ESKF::Predict(const double& dt, const ESKF::ProcessNoiseType& Q, const Vec3d& gyro, const Vec3d& acce) {
    Eigen::Matrix<double, 24, 1> f_ = x_.get_f(gyro, acce);  // 调用get_f 获取 速度 角速度 加速度
    Eigen::Matrix<double, 24, 23> f_x_ = x_.df_dx(acce);

    Eigen::Matrix<double, 24, 12> f_w_ = x_.df_dw();
    Eigen::Matrix<double, 23, process_noise_dim_> f_w_final;

    NavState x_before = x_;
    x_.oplus(f_, dt);

    F_x1_ = CovType::Identity();

    // set f_x_final
    CovType f_x_final;  // 23x23
    for (auto st : x_.vect_states_) {
        int idx = st.idx_;
        int dim = st.dim_;
        int dof = st.dof_;

        for (int i = 0; i < 23; i++) {
            for (int j = 0; j < dof; j++) {
                f_x_final(idx + j, i) = f_x_(dim + j, i);
            }
        }

        for (int i = 0; i < process_noise_dim_; i++) {
            for (int j = 0; j < dof; j++) {
                f_w_final(idx + j, i) = f_w_(dim + j, i);
            }
        }
    }

    Mat3d res_temp_SO3;
    Vec3d seg_SO3;
    for (auto st : x_.SO3_states_) {
        int idx = st.idx_;
        int dim = st.dim_;
        for (int i = 0; i < 3; i++) {
            seg_SO3(i) = -1 * f_(dim + i) * dt;
        }

        F_x1_.block<3, 3>(idx, idx) = math::exp(seg_SO3, 0.5).matrix();

        res_temp_SO3 = math::A_matrix(seg_SO3);
        for (int i = 0; i < state_dim_; i++) {
            f_x_final.template block<3, 1>(idx, i) = res_temp_SO3 * (f_x_.block<3, 1>(dim, i));
        }

        for (int i = 0; i < process_noise_dim_; i++) {
            f_w_final.template block<3, 1>(idx, i) = res_temp_SO3 * (f_w_.block<3, 1>(dim, i));
        }
    }

    Eigen::Matrix<double, 2, 3> res_temp_S2;
    Vec3d seg_S2;
    for (auto st : x_.S2_states_) {
        int idx = st.idx_;
        int dim = st.dim_;
        for (int i = 0; i < 3; i++) {
            seg_S2(i) = f_(dim + i) * dt;
        }

        SO3 res = math::exp(seg_S2, 0.5f);

        Vec2d vec = Vec2d::Zero();
        Eigen::Matrix<double, 2, 3> Nx = x_.grav_.S2_Nx_yy();
        Eigen::Matrix<double, 3, 2> Mx = x_before.grav_.S2_Mx(vec);

        F_x1_.block<2, 2>(idx, idx) = Nx * res.matrix() * Mx;

        Eigen::Matrix<double, 3, 3> x_before_hat = x_before.grav_.S2_hat();
        res_temp_S2 = -Nx * res.matrix() * x_before_hat * math::A_matrix(seg_S2).transpose();

        for (int i = 0; i < state_dim_; i++) {
            f_x_final.block<2, 1>(idx, i) = res_temp_S2 * (f_x_.block<3, 1>(dim, i));
        }
        for (int i = 0; i < process_noise_dim_; i++) {
            f_w_final.block<2, 1>(idx, i) = res_temp_S2 * (f_w_.block<3, 1>(dim, i));
        }
    }

    F_x1_ += f_x_final * dt;
    P_ = (F_x1_)*P_ * (F_x1_).transpose() + (dt * f_w_final) * Q * (dt * f_w_final).transpose();
}

/**
 * 原版的迭代过程中，收敛次数大于1才会结果，所以需要两次收敛。
 * 在未收敛时，实际上不会计算最近邻，也就回避了一次ObsModel的计算
 * 如果这边对每次迭代都计算最近邻的话，时间明显会变长一些，并不是非常合理。。
 *
 * @param obs
 * @param R
 */
void ESKF::Update(ESKF::ObsType obs, const double& R) {
    custom_obs_model_.valid_ = true;
    custom_obs_model_.converge_ = true;

    CovType P_propagated = P_;

    Eigen::Matrix<double, 23, 1> K_r;
    Eigen::Matrix<double, 23, 23> K_H;

    StateVecType dx_current = StateVecType::Zero();  // 本轮迭代的dx

    NavState start_x = x_;  // 迭代的起点
    NavState last_x = x_;

    int converged_times = 0;
    double last_lidar_res = 0;

    double init_res = 0.0;
    static double iterated_num = 0;
    static double update_num = 0;
    update_num += 1;
    for (int i = -1; i < maximum_iter_; i++) {
        custom_obs_model_.valid_ = true;

        /// 计算observation function，主要是residual_, h_x_, s_
        /// x_ 在每次迭代中都是更新的，线性化点也会更新
        if (obs == ObsType::LIDAR || obs == ObsType::WHEEL_SPEED_AND_LIDAR) {
            lidar_obs_func_(x_, custom_obs_model_);
        } else if (obs == ObsType::WHEEL_SPEED) {
            wheelspeed_obs_func_(x_, custom_obs_model_);
        } else if (obs == ObsType::ACC_AS_GRAVITY) {
            acc_as_gravity_obs_func_(x_, custom_obs_model_);
        } else if (obs == ObsType::GPS) {
            gps_obs_func_(x_, custom_obs_model_);
        } else if (obs == ObsType::BIAS) {
            bias_obs_func_(x_, custom_obs_model_);
        }

        if (use_aa_ && i > -1 && (obs == ObsType::LIDAR || obs == ObsType::WHEEL_SPEED_AND_LIDAR) &&
            custom_obs_model_.lidar_residual_mean_ >= last_lidar_res * 1.01) {
            x_ = last_x;
            break;
        }
        iterated_num += 1;

        if (!custom_obs_model_.valid_) {
            continue;
        }

        if (i == -1) {
            init_res = custom_obs_model_.lidar_residual_mean_;
            if (init_res < 1e-9) {
                init_res = 1e-9;  // 可能有零
            }
        }

        iterations_ = i + 2;  // i从-1开始计
        final_res_ = custom_obs_model_.lidar_residual_mean_ / init_res;

        int dof_measurement = custom_obs_model_.h_x_.rows();
        StateVecType dx = x_.boxminus(start_x);  // 当前x与起点之间的dx
        dx_current = dx;                         //

        P_ = P_propagated;

        /// 更新P 和 dx
        /// P = J*P*J^T
        /// dx = J * dx
        for (auto it : x_.SO3_states_) {
            int idx = it.idx_;
            Vec3d seg_SO3 = dx.block<3, 1>(idx, 0);
            Mat3d res_temp_SO3 = math::A_matrix(seg_SO3).transpose();  // 小块的J阵, SO3上的雅可比？

            dx_current.block<3, 1>(idx, 0) = res_temp_SO3 * dx.block<3, 1>(idx, 0);

            /// P 上面有SO3的行 进行转换
            for (int j = 0; j < state_dim_; j++) {
                P_.block<3, 1>(idx, j) = res_temp_SO3 * (P_.block<3, 1>(idx, j));
            }
            /// P 上面有SO3的列 进行转换
            for (int j = 0; j < state_dim_; j++) {
                P_.block<1, 3>(j, idx) = (P_.block<1, 3>(j, idx)) * res_temp_SO3.transpose();
            }
        }

        for (auto it : x_.S2_states_) {
            int idx = it.idx_;

            Vec2d seg_S2 = dx.block<2, 1>(idx, 0);

            Eigen::Matrix<double, 2, 3> Nx = x_.grav_.S2_Nx_yy();
            Eigen::Matrix<double, 3, 2> Mx = start_x.grav_.S2_Mx(seg_S2);
            Mat2d res_temp_S2 = Nx * Mx;

            dx_current.block<2, 1>(idx, 0) = res_temp_S2 * dx.block<2, 1>(idx, 0);

            for (int j = 0; j < state_dim_; j++) {
                P_.block<2, 1>(idx, j) = res_temp_S2 * (P_.block<2, 1>(idx, j));
            }

            for (int j = 0; j < state_dim_; j++) {
                P_.block<1, 2>(j, idx) = (P_.block<1, 2>(j, idx)) * res_temp_S2.transpose();
            }
        }

        /// 处理各类观测模型
        if (state_dim_ > dof_measurement) {
            Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(dof_measurement, state_dim_);
            // h_x_ 列数：雷达观测为12维紧凑块(pos/rot/外参)；轮速等观测为23维全布局(含vel)
            h_x_cur.topLeftCorner(dof_measurement, custom_obs_model_.h_x_.cols()) = custom_obs_model_.h_x_;
            custom_obs_model_.R_ = R * Eigen::MatrixXd::Identity(dof_measurement, dof_measurement);

            // 注：cr101@ece11e8 的轮速卡方软拒绝实测为负优化（轮速系统偏差使残差系统性偏大，
            // 门限压制有效更新），已移除；如需可参照 rk100 同提交找回
            const Eigen::MatrixXd S = h_x_cur * P_ * h_x_cur.transpose() + custom_obs_model_.R_;

            // K = P H^T S^-1 = (S^-1 H P)^T, P/S 对称, LDLT 求解避免显式求逆
            Eigen::MatrixXd K = (S.ldlt().solve(h_x_cur * P_)).transpose();
            K_r = K * custom_obs_model_.residual_;
            K_H = K * h_x_cur;
        } else {
            /// 纯雷达观测
            double R_inv = 1.0 / (R * dof_measurement);

            // HTRH = H^T R^-1 H
            Eigen::Matrix<double, 12, 12> HTH = custom_obs_model_.h_x_.transpose() * custom_obs_model_.h_x_;

            CovType P_temp = (P_ / R).inverse();  // P阵上面已经更新
            P_temp.block<12, 12>(0, 0) += HTH;    // Q in (38)
            CovType Q_inv = P_temp.inverse();     // Q inv

            // Q*H^T * R^-1 * r = K * r
            // <-- K ----->
            K_r = Q_inv.template block<23, 12>(0, 0) * custom_obs_model_.h_x_.transpose() * custom_obs_model_.residual_;

            // K_H = Q^-1 H^T R^-1 H
            //       <--  K     ->
            K_H.setZero();
            K_H.template block<23, 12>(0, 0) = Q_inv.template block<23, 12>(0, 0) * HTH;
        }

        // dx = Kr + (KH-I) dx
        dx_current = K_r + (K_H - Eigen::Matrix<double, 23, 23>::Identity()) * dx_current;

        // ==================== 雷达退化检测与处理 ====================
        // 参考 ct-lio 的 checkLocalizability: 用参与匹配的法向量分布做退化检测。
        // Htt = H^T H 的平移块 = Σ n_i n_i^T, 其最小特征值 = 法向量矩阵最小奇异值的平方,
        // 该值偏小说明这个方向的法向量分布"平坦"→ 该方向不可观(长走廊/稀疏区典型)。
        // ct-lio 只检测不处理, 这里往前走一步:
        //   1) 把修正量投影到可观测子空间(不可观方向不吃修正, 交给 IMU/轮速)
        //   2) 修正量硬限幅兜底(匹配落错误局部极小时, 实测单帧偏航修正可达 -57°)
        if ((obs == ObsType::LIDAR || obs == ObsType::WHEEL_SPEED_AND_LIDAR) && custom_obs_model_.h_x_.cols() >= 12 &&
            custom_obs_model_.h_x_.rows() >= 10) {
            const Eigen::MatrixXd& Hd = custom_obs_model_.h_x_;
            const Eigen::Matrix3d Htt = Hd.leftCols(3).transpose() * Hd.leftCols(3);
            const Eigen::Matrix3d Hrr = Hd.middleCols(3, 3).transpose() * Hd.middleCols(3, 3);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_tt(Htt);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_rr(Hrr);
            const bool ok_tt = (es_tt.info() == Eigen::Success);
            const bool ok_rr = (es_rr.info() == Eigen::Success);

            // λ_min / λ_avg 低于阈值 → 该方向退化
            constexpr double kDegRatio = 0.10;
            bool deg_tt = false, deg_rr = false;
            if (ok_tt) {
                double avg = Htt.trace() / 3.0;
                deg_tt = es_tt.eigenvalues()(0) < kDegRatio * (avg > 1e-12 ? avg : 1e-12);
            }
            if (ok_rr) {
                double avg = Hrr.trace() / 3.0;
                deg_rr = es_rr.eigenvalues()(0) < kDegRatio * (avg > 1e-12 ? avg : 1e-12);
            }

            Vec3d dp = dx_current.block<3, 1>(0, 0);
            Vec3d dr = dx_current.block<3, 1>(3, 0);
            if (deg_tt) {
                const Vec3d v0 = es_tt.eigenvectors().col(0);  // 不可观平移方向
                dp -= v0 * v0.dot(dp);
            }
            if (deg_rr) {
                const Vec3d v0 = es_rr.eigenvectors().col(0);  // 不可观旋转方向
                dr -= v0 * v0.dot(dr);
            }
            dx_current.block<3, 1>(0, 0) = dp;
            dx_current.block<3, 1>(3, 0) = dr;

            // 硬限幅兜底: 等比缩放, 保留修正方向但限制幅度
            constexpr double kMaxPosCorr = 0.5;   // m/帧
            constexpr double kMaxRotCorr = 0.10;  // rad/帧 (~5.7°)
            const double pos_n = dp.norm();
            const double rot_n = dr.norm();
            double scale = 1.0;
            if (pos_n > kMaxPosCorr) scale = kMaxPosCorr / pos_n;
            if (rot_n > kMaxRotCorr && kMaxRotCorr / rot_n < scale) scale = kMaxRotCorr / rot_n;
            if (scale < 1.0) { dx_current *= scale; }

            // 统计: 每 50 次调用汇总一次退化/限幅发生率与特征值（默认停用，需要复看发生率时放开）
            /*
            static int deg_cnt = 0;
            static int deg_tt_cnt = 0, deg_rr_cnt = 0, sat_cnt = 0;
            static double min_ratio_tt = 1e30, min_ratio_rr = 1e30;
            static double max_pos_corr = 0, max_rot_corr = 0;
            deg_cnt++;
            if (deg_tt) deg_tt_cnt++;
            if (deg_rr) deg_rr_cnt++;
            if (scale < 1.0) sat_cnt++;
            if (ok_tt) {
                double r = es_tt.eigenvalues()(0) / (Htt.trace() / 3.0 > 1e-12 ? Htt.trace() / 3.0 : 1e-12);
                if (r < min_ratio_tt) min_ratio_tt = r;
            }
            if (ok_rr) {
                double r = es_rr.eigenvalues()(0) / (Hrr.trace() / 3.0 > 1e-12 ? Hrr.trace() / 3.0 : 1e-12);
                if (r < min_ratio_rr) min_ratio_rr = r;
            }
            if (pos_n > max_pos_corr) max_pos_corr = pos_n;
            if (rot_n > max_rot_corr) max_rot_corr = rot_n;
            if (deg_cnt % 50 == 0) {
                LOG(WARNING) << "[DEG] n=" << deg_cnt << " deg_tt=" << deg_tt_cnt << " deg_rr=" << deg_rr_cnt
                             << " sat=" << sat_cnt << " | minRatio tt=" << min_ratio_tt
                             << " rr=" << min_ratio_rr << " | maxCorr pos=" << max_pos_corr
                             << "m rot=" << max_rot_corr * 57.29578 << "deg";
                deg_tt_cnt = deg_rr_cnt = sat_cnt = 0;
                min_ratio_tt = min_ratio_rr = 1e30;
                max_pos_corr = max_rot_corr = 0;
            }
            */
        }

        // check nan
        for (int j = 0; j < 23; ++j) {
            if (std::isnan(dx_current(j, 0))) {
                return;
            }
        }

        if (!use_aa_) {
            x_ = x_.boxplus(dx_current);
        } else {
            // 转到起点的线性空间
            x_ = x_.boxplus(dx_current);

            if (i == -1) {
                aa_.init(dx_current);  // 初始化AA
            } else {
                // 利用AA计算dx from start
                auto dx_all = x_.boxminus(start_x);
                auto new_dx_all = aa_.compute(dx_all);
                x_ = start_x.boxplus(new_dx_all);
            }
        }

        last_x = x_;

        // update last res
        last_lidar_res = custom_obs_model_.lidar_residual_mean_;
        custom_obs_model_.converge_ = true;

        for (int j = 0; j < 23; j++) {
            if (std::fabs(dx_current[j]) > limit_[j]) {
                custom_obs_model_.converge_ = false;
                break;
            }
        }

        if (custom_obs_model_.converge_) {
            converged_times++;
        }

        if (!converged_times && i == maximum_iter_ - 2) {
            custom_obs_model_.converge_ = true;
        }

        if (converged_times > 0 || i == maximum_iter_ - 1) {
            /// 结束条件：已经收敛
            /// 更新P阵, using (45)
            L_ = P_;
            Mat3d res_temp_SO3;
            Vec3d seg_SO3;
            for (auto it : x_.SO3_states_) {
                int idx = it.idx_;
                for (int j = 0; j < 3; j++) {
                    seg_SO3(j) = dx_current(j + idx);
                }

                res_temp_SO3 = math::A_matrix(seg_SO3).transpose();
                for (int j = 0; j < 23; j++) {
                    L_.block<3, 1>(idx, j) = res_temp_SO3 * (P_.block<3, 1>(idx, j));
                }

                for (int j = 0; j < 12; j++) {
                    K_H.block<3, 1>(idx, j) = res_temp_SO3 * (K_H.block<3, 1>(idx, j));
                }

                for (int j = 0; j < 23; j++) {
                    L_.block<1, 3>(j, idx) = (L_.block<1, 3>(j, idx)) * res_temp_SO3.transpose();
                    P_.block<1, 3>(j, idx) = (P_.block<1, 3>(j, idx)) * res_temp_SO3.transpose();
                }
            }

            Mat2d res_temp_S2;
            Vec2d seg_S2;
            for (auto it : x_.S2_states_) {
                int idx = it.idx_;

                for (int j = 0; j < 2; j++) {
                    seg_S2(j) = dx_current(j + idx);
                }

                Eigen::Matrix<double, 2, 3> Nx = x_.grav_.S2_Nx_yy();
                Eigen::Matrix<double, 3, 2> Mx = start_x.grav_.S2_Mx(seg_S2);
                res_temp_S2 = Nx * Mx;

                for (auto j = 0; j < 23; j++) {
                    L_.block<2, 1>(idx, j) = res_temp_S2 * (P_.block<2, 1>(idx, j));
                }

                for (auto j = 0; j < 12; j++) {
                    K_H.block<2, 1>(idx, j) = res_temp_S2 * (K_H.block<2, 1>(idx, j));
                }

                for (int j = 0; j < 23; j++) {
                    L_.block<1, 2>(j, idx) = (L_.block<1, 2>(j, idx)) * res_temp_S2.transpose();
                    P_.block<1, 2>(j, idx) = (P_.block<1, 2>(j, idx)) * res_temp_S2.transpose();
                }
            }

            P_ = L_ - K_H * P_;

            // 数值保护：P=(I-KH)P 非 Joseph 形式且 L_/P_ 混用会产生非对称矩阵，使下一次
            // S=HPH^T+R 失去正定，K 溢出、状态发散。此处对称化并钳制最小特征值，保证 P 半正定。
            constexpr double kCovEigenFloor = 1e-10;
            P_ = (0.5 * (P_ + P_.transpose())).eval();
            Eigen::SelfAdjointEigenSolver<CovType> cov_es(P_);
            if (cov_es.info() == Eigen::Success && cov_es.eigenvalues().minCoeff() < kCovEigenFloor) {
                P_ = cov_es.eigenvectors() * cov_es.eigenvalues().cwiseMax(kCovEigenFloor).asDiagonal() *
                     cov_es.eigenvectors().transpose();
            }

            break;
        }
    }
}

}  // namespace lightning