//
// Created by xiang on 24-5-13.
//

#ifndef MIAO_CAUCHY_H
#define MIAO_CAUCHY_H

#include "robust_kernel.h"

namespace lightning::miao {

/**
 * \brief Cauchy cost function
 *
 *  2     e
 * d  log(-- + 1)
 *         2
 *        d
 */
class RobustKernelCauchy : public RobustKernel {
   public:
    virtual void Robustify(double e2, Vector3& rho) const override {
        constexpr double eps = 1e-12;  // 数值保护：防止 delta=0 导致除零或 NaN
        double dsqr = delta_ * delta_ + eps;
        double dsqrReci = 1. / dsqr;
        double aux = dsqrReci * e2 + 1.0;

        rho[0] = dsqr * log(aux);
        rho[1] = 1. / aux;
        rho[2] = -dsqrReci * std::pow(rho[1], 2);
    }
};
}  // namespace lightning::miao

#endif  // MIAO_CAUCHY_H
