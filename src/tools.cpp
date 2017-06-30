#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	//check division by zero
	if(fabs(c1) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
			-(py/c1), (px/c1), 0, 0,
			py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;
}


VectorXd Tools::Cart2Polar(const VectorXd& x_state)
{
	VectorXd y(3);
	y << 0,0,0;
	if(x_state.size() != 4)
	{
		cout << "input size invalid, input must be of size = 4 {pos , velocity)";
		return y;
	}
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	float rho = sqrt(px*px+py*py);
	float phi = atan2(py,px);
	if(fabs(rho) < 0.0001){
		cout << "Cart2Polar () - Error - Division by Zero" << endl;
		return y;
	}
	float rho_dot = (px*vx + py*vy)/ rho;
	y = VectorXd(3);
	y << rho,phi,rho_dot;
	return y ;
}

VectorXd Tools::Polar2Cart(const VectorXd& x_state)
{
	VectorXd y(4);
	y << 0,0,0,0;
	if(x_state.size() != 3)
	{
		cout << "input size invalid, input must be of size = 3 {rho,phi,roh_dot)";
		return y;
	}
	//recover state parameters
	float rho = x_state(0);
	float phi = x_state(1);
	float rho_dot = x_state(2);

	float px = rho * cosf(phi);
	float py = rho * sinf(phi);
	float vx = rho_dot * cosf(phi);
	float vy = rho_dot * sinf(phi);
	y = VectorXd(4);
	y << px,py,vx,vy;
	return y;
}