#include <iostream>
#include <vector>
#include <Eigen\Dense>

#include "kalman.hpp"

std::vector<double> data = {
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	18,
	24,
	31,
	33,
	31,
	20,
	15,
	29,
	35,
	32,
	24,
	23,
	23,
	15,
	12,
	12,
	11,
	10,
	10,
	12,
	12,
	12,
	11,
	11,
	17,
	20,
	20,
	22,
	22,
	25,
	25,
	24,
	27,
	27,
	29,
	30,
	32,
	32,
	33,
	36,
	38,
	39,
	41,
	44,
	45,
	45,
	47,
	51,
	52,
	50,
	51,
	54,
	55,
	55,
	55,
	57,
	59,
	61,
	64,
	63,
	63,
	67,
	66,
	64,
	64,
	66,
	68,
	71,
	72,
	76,
	74,
	75,
	79,
	79,
	78,
	69,
	74,
	79,
	76,
	71,
	77,
	86,
	82,
	87,
	89,
	84,
	85,
	83,
	82,
	82,
	83,
	87,
	85,
	83,
	91,
	86,
	84,
	90,
	83,
	88,
	87,
	84,
	85,
	84,
	87,
	85,
	84,
	90,
	86,
	90,
	89,
	87,
	90,
	89,
	89,
	84,
	89,
	87,
	84,
	88,
	84,
	86,
	87,
	84,
	87,
	86,
	93,
	86,
	91,
	90,
	92,
	87,
	89,
	91,
	87,
	93,
	87,
	90,
	85,
	84,
	86,
	84,
	89,
	89,
	85,
	88,
	82,
	88,
	87,
	80,
	85,
	87,
	85,
	87,
	87,
	91,
	84,
	85,
	90,
	87,
	93,
	89,
	90,
	86,
	90,
	84,
	82,
	86,
	81,
	81,
	87,
	81,
	81,
	86,
	81,
	84,
	87,
	87,
	89,
	86,
	90,
	84,
	85,
	89,
	81,
	85,
	87,
	84,
	84,
	85,
	83,
	84,
	81,
	82,
	85,
	84,
	90,
	89,
	84,
	89,
	85,
	85,
	86,
	82,
	84,
	86,
	83,
	83,
	84,
	82,
	83,
	84,
	84,
	89,
	84,
	90,
	89,
	84,
	90,
	84,
	80,
	84,
	84,
	84,
	83,
	82,
	84,
	82,
	82,
	86,
	85,
	85,
	92,
	85,
	88,
	87,
	86,
	85,
	84,
	85,
	84
};

int measurementStep = 0;
double pos = 0;

Eigen::VectorXd generateMeasurements(double dt)
{
	Eigen::VectorXd m(6);

	double din = data[measurementStep];
	pos += din * dt;

	m << 0.0, 0.0, pos, din, 0.0, 0.0;
	measurementStep++;

	return m;
}

int main(int argc, char* argv[])
{
	int n = 6; // Number of states
	int m = 6; // Number of measurements

	double t0 = 0; // Initial time
	double dt = 1; // Time step

	Eigen::MatrixXd A(n, n); // State transition matrix
	Eigen::MatrixXd B(n, n); // Control matrix
	Eigen::MatrixXd H(m, n); // Observation matrix
	Eigen::MatrixXd Q(n, n); // Process noise covariance
	Eigen::MatrixXd R(m, m); // Measurement noise covariance
	Eigen::MatrixXd P(n, n); // Estimate error covariance

	A << 1, dt, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, dt, 0, 0,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, dt,
		0, 0, 0, 0, 0, 1;

	B << 0, 0, 0, 0, 0, 0,
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
	Q << q_val, q_val, .0, .0, .0, .0,
		.0, q_val, .0, .0, .0, .0,
		.0, .0, q_val, q_val, .0, .0,
		.0, .0, .0, q_val, .0, .0,
		.0, .0, .0, .0, q_val, q_val,
		.0, .0, .0, .0, .0, q_val;

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

	// List of noisy position measurements (z)
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
	u << 0, 0, 0, 0, 0, 0;

	// Test measurement
	Eigen::VectorXd testMeasurement(n);
	testMeasurement << 1, 0, 1, 0, 0, 0.01;
	//kf.update(testMeasurement, u);

	for (int i = 0; i < data.size(); i++)
	{
		Eigen::VectorXd din = generateMeasurements(dt);
		kf.update(din);
		//std::cout << din[3] << "," << kf.state()[2] << "," << kf.state()[3] << std::endl;
	}

	// Print new state
	//std::cout << kf.state() << std::endl;

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
