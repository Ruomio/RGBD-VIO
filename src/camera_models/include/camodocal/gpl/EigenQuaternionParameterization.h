#ifndef EIGENQUATERNIONPARAMETERIZATION_H
#define EIGENQUATERNIONPARAMETERIZATION_H

// #include "ceres/local_parameterization.h"
#include <ceres/manifold.h>
#include <eigen3/Eigen/Eigen>


namespace camodocal
{

class EigenQuaternionParameterization : public ceres::Manifold
{
public:
    virtual ~EigenQuaternionParameterization() {}
    // virtual bool Plus(const double* x,
    //                   const double* delta,
    //                   double* x_plus_delta) const;
    // virtual bool ComputeJacobian(const double* x,
    //                              double* jacobian) const;
    // virtual int GlobalSize() const { return 4; }
    // virtual int LocalSize() const { return 3; }

    virtual int AmbientSize() const { return 4; };
    virtual int TangentSize() const { return 3; };
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool PlusJacobian(const double *x, double *jacobian) const;
    virtual bool RightMultiplyByPlusJacobian(const double *x, const int num_rows, const double *ambient_matrix, double *tangent_matrix) const;
    virtual bool Minus(const double *y, const double *x, double *y_minus_x) const;
    virtual bool MinusJacobian(const double *x, double *jacobian) const;

private:
    template<typename T>
    void EigenQuaternionProduct(const T z[4], const T w[4], T zw[4]) const;
};


template<typename T>
void
EigenQuaternionParameterization::EigenQuaternionProduct(const T z[4], const T w[4], T zw[4]) const
{
    zw[0] = z[3] * w[0] + z[0] * w[3] + z[1] * w[2] - z[2] * w[1];
    zw[1] = z[3] * w[1] - z[0] * w[2] + z[1] * w[3] + z[2] * w[0];
    zw[2] = z[3] * w[2] + z[0] * w[1] - z[1] * w[0] + z[2] * w[3];
    zw[3] = z[3] * w[3] - z[0] * w[0] - z[1] * w[1] - z[2] * w[2];
}

}

#endif
