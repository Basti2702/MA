/*
 * PointCell.h
 *
 *  Created on: 04.08.2016
 *      Author: Sebastian Reinhart
 */

#ifndef POINTCELL_H_
#define POINTCELL_H_
#include "Matrix.cuh"
#include "data.cuh"
#include <math.h>
#include <cuda.h>

class PointCell {
public:
	PointCell();
	virtual ~PointCell();

	Matrix<double> stateVector; //x = [x , y, theta(rad), velocity(m/s), yaw rate(rad/s)]
	Matrix<double> stateCopy;

	double subInvtl;

	void predict();
	void update(Matrix<double> newState);
	void setID();
	int getID();
	void setID(int id);
	__host__ __device__ double getX();
	__host__ __device__ double getY();
	__host__ __device__ double getTheta();
	__host__ __device__ double getVelocity();
	__host__ __device__ double getPhi();

	__host__ __device__ void setX(double x);
	__host__ __device__ void setY(double y);
	__host__ __device__ void setTheta(double theta);
	__host__ __device__ void setVelocity(double velocity);
	__host__ __device__ void setPhi(double phi);
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

	IdentityMatrix<double> I;

	int ID;
};

#endif /* POINTCELL_H_ */
