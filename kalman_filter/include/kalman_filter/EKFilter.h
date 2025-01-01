#ifndef EKFILTER_H
#define EKFILTER_H

#include <cnoid/EigenTypes>
#include <cnoid/EigenUtil>
#include <iostream>

namespace cnoid {
    typedef Eigen::Matrix<double, 7, 7> Matrix7;
    typedef Eigen::Matrix<double, 7, 1> Vector7;
}; // namespace cnoid

class EKFilter {
  public:
    EKFilter()
        : P(cnoid::Matrix7::Identity() * 0.1), Q(cnoid::Matrix3::Identity() * 0.001),
          R(cnoid::Matrix3::Identity() * 0.03), g_vec(cnoid::Vector3(0.0, 0.0, 9.80665)),
          z_k(cnoid::Vector3(0.0, 0.0, 9.80665)), min_mag_thre_acc(0.005), max_mag_thre_acc(0.05),
          min_mag_thre_gyro(0.0075), max_mag_thre_gyro(0.035) {
        x << 1, 0, 0, 0, 0, 0, 0;
    }

    cnoid::Vector7 getx() const { return x; }

    void calcOmega(cnoid::Matrix4 &omega, const cnoid::Vector3 &w) const {
        /* \dot{q} = \frac{1}{2} omega q */
        omega << 0, -w[0], -w[1], -w[2], w[0], 0, w[2], -w[1], w[1], -w[2], 0, w[0], w[2], w[1], -w[0], 0;
    }

    void calcPredictedState(cnoid::Vector7 &_x_a_priori, const cnoid::Vector4 &q, const cnoid::Vector3 &gyro,
                            const cnoid::Vector3 &drift) const {
        /* x_a_priori = f(x, u) */
        cnoid::Vector4 q_a_priori;
        cnoid::Vector3 gyro_compensated = gyro - drift;
        cnoid::Matrix4 omega;
        calcOmega(omega, gyro_compensated);
        q_a_priori            = q + dt / 2 * omega * q;
        _x_a_priori.head<4>() = q_a_priori.normalized();
        _x_a_priori.tail<3>() = drift;
    }

    void calcF(cnoid::Matrix7 &F, const cnoid::Vector4 &q, const cnoid::Vector3 &gyro,
               const cnoid::Vector3 &drift) const {
        F                               = cnoid::Matrix7::Identity();
        cnoid::Vector3 gyro_compensated = gyro - drift;
        cnoid::Matrix4 omega;
        calcOmega(omega, gyro_compensated);
        F.block<4, 4>(0, 0) += dt / 2 * omega;
        F.block<4, 3>(0, 4) << +q[1], +q[2], +q[3], -q[0], +q[3], -q[2], -q[3], -q[0], +q[1], +q[2], -q[1], -q[0];
        F.block<4, 3>(0, 4) *= dt / 2;
    }

    void calcPredictedCovariance(cnoid::Matrix7 &_P_a_priori, const cnoid::Matrix7 &F, const cnoid::Vector4 &q) const {
        /* P_a_priori = F P F^T + V Q V^T */
        Eigen::Matrix<double, 4, 3> V_upper;
        V_upper << -q[1], -q[2], -q[3], +q[0], -q[3], +q[2], +q[3], +q[0], -q[1], -q[2], +q[1], +q[0];
        V_upper *= dt / 2;
        cnoid::Matrix7 VQVt    = cnoid::Matrix7::Zero();
        VQVt.block<4, 4>(0, 0) = V_upper * Q * V_upper.transpose();
        _P_a_priori            = F * P * F.transpose() + VQVt;
    }

    Eigen::Vector3d calcAcc(const cnoid::Vector4 &q) const {
        cnoid::Quaternion q_tmp(q[0], q[1], q[2], q[3]);
        cnoid::Vector3 acc = q_tmp.conjugate()._transformVector(g_vec);
        return acc;
    }

    void calcH(Eigen::Matrix<double, 3, 7> &H, const cnoid::Vector4 &q) const {
        double w = q[0], x = q[1], y = q[2], z = q[3];
        H << -y, +z, -w, +x, 0, 0, 0, +x, +w, +z, +y, 0, 0, 0, +w, -x, -y, +z, 0, 0, 0;
        H *= 2 * g_vec[2];
    }

    Eigen::Vector3d calcMeasurementResidual(const cnoid::Vector3 &acc_measured, const cnoid::Vector4 &q) const {
        /* y = z - h(x) */
        cnoid::Vector3 y = acc_measured - calcAcc(q);
        return y;
    }


    void prediction(const cnoid::Vector3 &u) {
        cnoid::Vector4 q     = x.head<4>();
        cnoid::Vector3 drift = x.tail<3>();
        cnoid::Matrix7 F;
        calcF(F, q, u, drift);
        cnoid::Vector7 x_tmp;
        calcPredictedState(x_tmp, q, u, drift);
        x_a_priori = x_tmp;
        cnoid::Matrix7 P_tmp;
        calcPredictedCovariance(P_tmp, F, q);
        P_a_priori = P_tmp;
    }

    void correction(const cnoid::Vector3 &z, const cnoid::Matrix3 &fuzzyR) {
        cnoid::Vector4 q_a_priori = x_a_priori.head<4>();
        Eigen::Matrix<double, 3, 7> H;
        z_k              = z;
        cnoid::Vector3 y = calcMeasurementResidual(z, q_a_priori);
        calcH(H, q_a_priori);
        cnoid::Matrix3 S              = H * P_a_priori * H.transpose() + fuzzyR;
        Eigen::Matrix<double, 7, 3> K = P_a_priori * H.transpose() * S.inverse();
        cnoid::Vector7 x_tmp          = x_a_priori + K * y;
        x.head<4>()                   = x_tmp.head<4>().normalized(); /* quaternion */
        x.tail<3>()                   = x_tmp.tail<3>();              /* bias */
        P                             = (cnoid::Matrix7::Identity() - K * H) * P_a_priori;
    }

    void printAll() const {
        std::cerr << "x" << std::endl << x << std::endl;
        std::cerr << "x_a_priori" << std::endl << x_a_priori << std::endl;
        std::cerr << "P" << std::endl << P << std::endl << std::endl;
        std::cerr << "P_a_priori" << std::endl << P_a_priori << std::endl << std::endl;
    }


    // Basically Equation (23), (24) and (25) in the paper [1]
    // [1] Chul Woo Kang and Chan Gook Park. Attitude estimation with accelerometers and gyros using fuzzy tuned Kalman
    // filter.
    //     In European Control Conference, 2009.
    void calcRWithFuzzyRule(cnoid::Matrix3 &fuzzyR, const cnoid::Vector3 &acc, const cnoid::Vector3 &gyro) const {
        double alpha = std::min(std::fabs(acc.norm() - g_vec.norm()) / g_vec.norm(), 0.1);
        double beta  = std::min(gyro.norm(), 0.05);
        double large_mu_acc =
            std::max(std::min((alpha - min_mag_thre_acc) / (max_mag_thre_acc - min_mag_thre_acc), 1.0), 0.0);
        double large_mu_gyro =
            std::max(std::min((beta - min_mag_thre_gyro) / (max_mag_thre_gyro - min_mag_thre_gyro), 1.0), 0.0);
        double w1, w2, w3, w4;
        w1 = (1.0 - large_mu_acc) * (1.0 - large_mu_gyro);
        w2 = (1.0 - large_mu_acc) * large_mu_gyro;
        w3 = large_mu_acc * (1.0 - large_mu_gyro);
        w4 = large_mu_acc * large_mu_gyro;
        double z =
            (w1 * 0.0 + w2 * (3.5 * alpha + 8.0 * beta + 0.5) + w3 * (3.5 * alpha + 8.0 * beta + 0.5) + w4 * 1.0) /
            (w1 + w2 + w3 + w4);
        double k1 = 400;
        fuzzyR    = R + k1 * z * z * cnoid::Matrix3::Identity();
    };

    void main_one(cnoid::Vector3 &rpy, const cnoid::Vector3 &acc, const cnoid::Vector3 &gyro) {
        cnoid::Matrix3 fuzzyR;
        calcRWithFuzzyRule(fuzzyR, acc, gyro);
        prediction(gyro);
        correction(acc, fuzzyR);
        /* ekf_filter.printAll(); */
        cnoid::Quaternion q(x[0], x[1], x[2], x[3]);
        rpy = cnoid::rpyFromRot(q.toRotationMatrix());
    };

    void main_one(cnoid::Quaternion &q, const cnoid::Vector3 &acc, const cnoid::Vector3 &gyro) {
        cnoid::Matrix3 fuzzyR;
        calcRWithFuzzyRule(fuzzyR, acc, gyro);
        prediction(gyro);
        correction(acc, fuzzyR);
        /* ekf_filter.printAll(); */
        cnoid::Quaternion xq(x[0], x[1], x[2], x[3]);
        q = xq.normalized();
    };

    void setdt(const double _dt) { dt = _dt; };
    void resetKalmanFilterState() {
        cnoid::Quaternion tmp_q;
        tmp_q.setFromTwoVectors(z_k, g_vec);
        x << tmp_q.w(), tmp_q.x(), tmp_q.y(), tmp_q.z(), 0, 0, 0;
    };

  private:
    cnoid::Vector7 x, x_a_priori;
    cnoid::Matrix7 P, P_a_priori;
    cnoid::Matrix3 Q, R;
    cnoid::Vector3 g_vec, z_k;
    double dt;
    double min_mag_thre_acc, max_mag_thre_acc, min_mag_thre_gyro, max_mag_thre_gyro;
};

#endif /* EKFILTER_H */
