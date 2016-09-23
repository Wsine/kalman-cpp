#include <iostream>
#include <stdexcept>

#include "kalman.hpp"

KalmanFilter::KalmanFilter(
  double dt,
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& B,
  const Eigen::MatrixXd& H,
  const Eigen::MatrixXd& Q,
  const Eigen::MatrixXd& R,
  const Eigen::MatrixXd& P):
  A(A), B(B), H(H), Q(Q), R(R), P0(P),
  m(H.rows()), n(A.rows()), dt(dt), initialized(false),
  I(n, n), x_hat(n), x_hat_new(n), u(n)
{
  I.setIdentity();
}

void KalmanFilter::init()
{
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  u = u.Zero(B.cols());
  initialized = true;
}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0)
{
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  u = u.Zero(B.cols());
  initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& z)
{
  if(!initialized)
    throw std::runtime_error("Cannot update: Kalman filter not initialized.");

  x_hat_new = A * x_hat + B * u;
  P = A*P*A.transpose() + Q;
  K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
  x_hat_new += K * (z - H*x_hat_new);
  P = (I - K*H)*P;
  x_hat = x_hat_new;

  t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& z, const Eigen::VectorXd& u)
{
	this->u = u;
	update(z);
}

void KalmanFilter::update(const Eigen::VectorXd& z, const double dt, const Eigen::MatrixXd A)
{
  this->A = A;
  this->dt = dt;
  update(z);
}
