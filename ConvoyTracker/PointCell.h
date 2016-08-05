/*
 * PointCell.h
 *
 *  Created on: 04.08.2016
 *      Author: basti
 */

#ifndef POINTCELL_H_
#define POINTCELL_H_
#include "Matrix.h"
#include "data.h"
#include <math.h>

class PointCell {
public:
	PointCell();
	virtual ~PointCell();

	Matrix<double> stateVector; //x = [x , y, theta(rad), velocity(m/s), yaw rate(rad/s)]
	void predict();
	void update(Matrix<double> newState);
	void setID();
	int getID();
	void setID(int id);
private:

	//necessary matrices and functions for kalman filtering
	// f = [x + (v/phi) * (-sin(theta) + sin(phi*T + theta)
	//	  y + (v/phi) * (cos(theta) - cos(phi*T + theta)
	//      phi*T + theta
	//	  v
	//      phi]
	void computeF(Matrix<double>& state);
	void computeCovarianceF(Matrix<double> state, Matrix<double>& F);

	Matrix<double> F; //covariance matrix of f:
	Matrix<double> P; //uncertainty
	Matrix<double> Q; //process noise
	Matrix<double> H; //covariance of new measurement
	Matrix<double> R; //measurement noise, in our case the same as process noise
	Matrix<double> K;
	Matrix<double> stateCopy;
	IdentityMatrix<double> I;

	int ID;
};

#endif /* POINTCELL_H_ */
