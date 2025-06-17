#include "camodocal/gpl/EigenQuaternionParameterization.h"

#include <cmath>

namespace camodocal
{


    bool EigenQuaternionParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const {

        Eigen::Map<const Eigen::Vector3d> p_x(x);
        Eigen::Map<const Eigen::Quaterniond> q_x(x + 3);

        Eigen::Map<const Eigen::Vector3d> p_delta(delta);

        Eigen::Vector3d rotvec = Eigen::Map<const Eigen::Vector3d>(delta + 3);
        Eigen::Quaterniond q_delta(Eigen::AngleAxisd(rotvec.norm(), rotvec.normalized()));

        Eigen::Map<Eigen::Vector3d> p_x_plus(x_plus_delta);
        Eigen::Map<Eigen::Quaterniond> q_x_plus(x_plus_delta + 3);

        p_x_plus = p_x + p_delta;
        q_x_plus = (q_x * q_delta).normalized();

        return true;
    }

    bool EigenQuaternionParameterization::PlusJacobian(const double *x, double *jacobian) const {

        Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> jaco(jacobian);

        jaco.topRows<6>().setIdentity();
        jaco.bottomRows<1>().setZero();

        return true;
    }

    bool EigenQuaternionParameterization::RightMultiplyByPlusJacobian(const double *x, const int num_rows, const double *ambient_matrix, double *tangent_matrix) const {
        // 1. get plus_jacobian
        Eigen::Matrix<double, 4, 3, Eigen::RowMajor> plus_jacobian;
        if (!PlusJacobian(x, plus_jacobian.data())) {
            return false;
        }

        // 2. exec: tangent_matrix = ambient_matrix * plus_jacobian
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 4, Eigen::RowMajor>>
            ambient_mat(ambient_matrix, num_rows, 4);
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>
            tangent_mat(tangent_matrix, num_rows, 3);

        tangent_mat = ambient_mat * plus_jacobian;

        return true;

    }

    bool EigenQuaternionParameterization::Minus(const double *y, const double *x, double *y_minus_x) const {

        Eigen::Map<const Eigen::Vector3d> p_y(y);
        Eigen::Map<const Eigen::Quaterniond> q_y(y + 3);

        Eigen::Map<const Eigen::Vector3d> p_x(x);
        Eigen::Map<const Eigen::Quaterniond> q_x(x + 3);

        Eigen::Map<Eigen::Vector3d> p_y_minus_x(y_minus_x);
        Eigen::Map<Eigen::Vector3d> q_y_minus_x(y_minus_x + 3);

        p_y_minus_x = p_y - p_x;

        Eigen::AngleAxisd axisd((q_x.inverse() * q_y).normalized());
        q_y_minus_x = axisd.angle() * axisd.axis();

        return true;
    }

    bool EigenQuaternionParameterization::MinusJacobian(const double *x, double *jacobian) const {

        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jaco(jacobian);

        jaco.rightCols<3>().setIdentity();
        jaco.leftCols<1>().setZero();

        return true;
    }

// bool
// EigenQuaternionParameterization::Plus(const double* x,
//                                       const double* delta,
//                                       double* x_plus_delta) const
// {
//     const double norm_delta =
//         sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
//     if (norm_delta > 0.0)
//     {
//         const double sin_delta_by_delta = (sin(norm_delta) / norm_delta);
//         double q_delta[4];
//         q_delta[0] = sin_delta_by_delta * delta[0];
//         q_delta[1] = sin_delta_by_delta * delta[1];
//         q_delta[2] = sin_delta_by_delta * delta[2];
//         q_delta[3] = cos(norm_delta);
//         EigenQuaternionProduct(q_delta, x, x_plus_delta);
//     }
//     else
//     {
//         for (int i = 0; i < 4; ++i)
//         {
//             x_plus_delta[i] = x[i];
//         }
//     }
//     return true;
// }
//
// bool
// EigenQuaternionParameterization::ComputeJacobian(const double* x,
//                                                  double* jacobian) const
// {
//     jacobian[0] =  x[3]; jacobian[1]  =  x[2]; jacobian[2]  = -x[1];  // NOLINT
//     jacobian[3] = -x[2]; jacobian[4]  =  x[3]; jacobian[5]  =  x[0];  // NOLINT
//     jacobian[6] =  x[1]; jacobian[7] = -x[0]; jacobian[8] =  x[3];  // NOLINT
//     jacobian[9] = -x[0]; jacobian[10]  = -x[1]; jacobian[11]  = -x[2];  // NOLINT
//     return true;
// }

}
