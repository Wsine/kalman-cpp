/**
 * Test for the KalmanFilter class with 1D projectile motion.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <iostream>
#include <vector>
#include <D:\Documents\C++_Programs\eigen\Eigen\Dense>

#include "kalman.hpp"

int main(int argc, char* argv[]) {

  int n = 6; // Number of states
  int m = 6; // Number of measurements

  double t0 = 0;      // Initial time
  double dt = 1.0/30; // Time step

  Eigen::MatrixXd A(n, n); // State transition matrix
  Eigen::MatrixXd B(n, n); // Control matrix
  Eigen::MatrixXd H(m, n); // Observation matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  A << 1,dt, 0, 0, 0, 0,
	   0, 1, 0, 0, 0, 0,
	   0, 0, 1,dt, 0, 0,
	   0, 0, 0, 1, 0, 0,
	   0, 0, 0, 0, 1,dt,
	   0, 0, 0, 0, 0, 1;

  B << 1, 0, 0, 0, 0, 0,
	   0, 0, 0, 0, 0, 0,
	   0, 0, 0, 0, 0, 0,
	   0, 0, 0, 0, 0, 0,
	   0, 0, 0, 0, 0, 0,
	   0, 0, 0, 0, 0, 0;

  H << 1, 0, 0, 0, 0, 0,
	   0, 1, 0, 0, 0, 0,
	   0, 0, 1, 0, 0, 0,
	   0, 0, 0, 1, 0, 0,
	   0, 0, 0, 0, 1, 0,
	   0, 0, 0, 0, 0, 1;

  double q_val = 0.05;
  Q << q_val, q_val,    .0,    .0,    .0,    .0,
	    .0,   q_val,    .0,    .0,    .0,    .0,
	    .0,    .0,   q_val, q_val,    .0,    .0,
		.0,    .0,      .0, q_val,    .0,    .0,
		.0,    .0,      .0,    .0, q_val, q_val,
		.0,    .0,      .0,    .0,    .0, q_val;

  R << .2, .0, .0, .0, .0, .0,
	   .0, .2, .0, .0, .0, .0,
	   .0, .0, .2, .0, .0, .0,
	   .0, .0, .0, .2, .0, .0,
	   .0, .0, .0, .0, .2, .0,
	   .0, .0, .0, .0, .0, .2;

  P << 1, 0, 0, 0, 0, 0,
	   0, 1, 0, 0, 0, 0,
	   0, 0, 1, 0, 0, 0,
	   0, 0, 0, 1, 0, 0,
	   0, 0, 0, 0, 1, 0,
	   0, 0, 0, 0, 0, 1;

  /*std::cout << "A: \n" << A << "\n" << std::endl;
  std::cout << "C: \n" << C << "\n" << std::endl;
  std::cout << "Q: \n" << Q << "\n" << std::endl;
  std::cout << "R: \n" << R << "\n" << std::endl;
  std::cout << "P: \n" << P << "\n" << std::endl;*/

  // Construct the filter
  KalmanFilter kf(dt, A, B, H, Q, R, P);

  // List of noisy position measurements (y)
  std::vector<double> measurements = {
      1.04202710058, 1.10726790452, 1.2913511148, 1.48485250951, 1.72825901034,
      1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
      2.57709350237, 2.6682215744, 2.51641839428, 2.76034056782, 2.88131780617,
      2.88373786518, 2.9448468727, 2.82866600131, 3.0006601946, 3.12920591669,
      2.858361783, 2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
      2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249, 2.23464791825,
      2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
      1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913, 0.678547664519,
      0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013, -0.602973173813
  };

  // Initial state should be at home, facing foward, not moving
  Eigen::VectorXd x0(n);
  x0 << 0, 0, 0, 0, 0, 0;
  kf.init(t0, x0);

  // Test control vector
  Eigen::VectorXd u(n);
  u << 5, 0, 0, 0, 0, 0;

  // Test measurement
  Eigen::VectorXd testMeasurement(n);
  testMeasurement << 1, 0, 1, 0, 0, 0.01;
  kf.update(testMeasurement, u);

  // Print new state
  std::cout << kf.state() << std::endl;

  //// Feed measurements into filter, output estimated states
  //double t = 0;
  //Eigen::VectorXd y(m);

  //std::cout << "Time,Measurement,Estimation\n" << std::endl;
  ////std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << std::endl;
  //
  //for(int i = 0; i < measurements.size(); i++)
  //{
  //  t += dt;
  //  y << measurements[i];
  //  kf.update(y);
  //  std::cout << t << "," << y.transpose() << "," << kf.state()[0] << "," << kf.state()[1] << "," << kf.state()[2] << std::endl;
  //}

  return 0;
}
