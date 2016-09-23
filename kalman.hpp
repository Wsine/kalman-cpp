#include <Eigen\Dense>

#pragma once

class KalmanFilter
{
public:
  /**
  * Constructor.
  *
  * @param dt Time step
  * @param A State transition matrix
  * @param B Control matrix
  * @param H Observation matrix
  * @param Q Process noise covariance
  * @param R Measurement noise covariance
  * @param P Estimate error covariance
  *
  * @return Ready KalmanFilter
  */
  KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P
  );

  /**
  * Initialize the filter with initial states as zero.
  */
  void init();

  /**
  * Initialize the filter.
  *
  * @param t0 Initial time
  * @param x0 Initial state
  */
  void init(double t0, const Eigen::VectorXd& x0);

  /**
  * Update the estimated state based on measured values. The time step is
  * assumed to remain constant. The control vector is assumed to be zero.
  *
  * @param z Measurement vector
  */
  void update(const Eigen::VectorXd& z);

  /**
  * Update the estimated state based on measured values. The time step is
  * assumed to remain constant.
  *
  * @param z Measurement vector
  * @parma u Control vector
  */
  void update(const Eigen::VectorXd& z, const Eigen::VectorXd& u);

  /**
  * Update the estimated state based on measured values, using the given time
  * step and dynamics matrix.
  *
  * @param z Measurement vector
  * @param dt Time step
  * @param A State transition matrix
  */
  void update(const Eigen::VectorXd& z, const double dt, const Eigen::MatrixXd A);

  /**
  * Returns the current state estimation.
  *
  * @return Current state estimation
  */
  const Eigen::VectorXd& state() const { return x_hat; }

  /**
   * Returns the current time.
   *
   * @return Current time
   */
  const double time() const { return t; }
private:
  // Matrices for computation
  Eigen::MatrixXd A, B, H, Q, R, P, K, P0;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;

  // Control vector
  Eigen::VectorXd u;
};
