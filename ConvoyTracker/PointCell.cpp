/*
 * PointCell.cpp
 *
 *  Created on: 04.08.2016
 *      Author: basti
 */

#include "PointCell.h"


PointCell::PointCell()
{
	stateVector = Matrix<double>(5,1); //values get filled in by DataReader
	P = Matrix<double>(5,5);
	//initialize P
	//values for position is trusted more than value for orientation and even more than
	P.put(0,0, 2); //x
	P.put(1,1, 2); //y
	P.put(2,2, 10); //theta
	P.put(3,3, 10);//v
	P.put(4,4, 10);//phi

	Q = Matrix<double>(5,5);
	//values taken from http://nbviewer.jupyter.org/github/balzer82/Kalman/blob/master/Extended-Kalman-Filter-CTRV.ipynb?create=1
	Q.put(0,0, 0.000006); //x
	Q.put(1,1, 0.000006); //y
	Q.put(2,2, 0.0004); //theta
	Q.put(3,3, 0.03097);//v
	Q.put(4,4, 0.0004);//phi

	R = Matrix<double>(5,5);
	R.put(0,0, 0.36); //x
	R.put(1,1, 0.36); //y
	R.put(2,2, 0.5); //theta
	R.put(3,3, 0.1);//v
	R.put(4,4, 0.1);//phi

	H = IdentityMatrix<double>(5,5);
	I = IdentityMatrix<double>(5,5);
	F = IdentityMatrix<double>(5,5);
	F.put(2,4, TIMESTAMP);
}

PointCell::~PointCell() {
	// TODO Auto-generated destructor stub
}
/**
 * Performs the kalman filter prediction step
 */
void PointCell::predict()
{
	stateCopy = stateVector;
	computeF(stateVector);
	computeCovarianceF(stateVector, F);
	P = F*P*F.getTranspose() + Q;
}
/*Performs the kalman filter update step
 * @param newState only continues position informations, velocity and yaw rate has to be computed first
 */
void PointCell::update(Matrix<double> newState)
{
	double velocity, phi;
	double xNew = newState.get(0,0);
	double yNew = newState.get(1,0);
	double thetaNew = newState.get(2,0);

	double x = stateCopy.get(0,0);
	double y = stateCopy.get(1,0);
	double theta = stateCopy.get(2,0);
	velocity = sqrt((xNew - x) * (xNew - x) + (yNew - y)*(yNew - y)) / TIMESTAMP;
	phi = (thetaNew-theta) / TIMESTAMP;

	stateVector.put(3,0,velocity);
	stateVector.put(4,0,phi);
	newState.put(3,0,velocity);
	newState.put(4,0,phi);

	Matrix<double> S = H*P*H.getTranspose() +R;
	K = (P*H.getTranspose()) * S.getInverse();
	stateVector = stateVector + (K*(newState - stateVector));
	P = (I - (K*H))*P;
}

int PointCell::getID()
{
	return ID;
}

void PointCell::setID(int id)
{
	ID = id;
}

// f = [x + (v/phi) * (-sin(theta) + sin(phi*T + theta)
//	  y + (v/phi) * (cos(theta) - cos(phi*T + theta)
//      phi*T + theta
//	  v
//      phi]
void PointCell::computeF(Matrix<double>& state)
{
	double x = state.get(0,0);
	double y = state.get(1,0);
	double theta = state.get(2,0);
	double velocity = state.get(3,0);
	double phi = state.get(4,0);

	double predictedX, predictedY, predictedTheta, predictedVel,predictedPhi;

	if(phi > 0.0001)
	{
		predictedX = (velocity/phi) * (sin(phi*TIMESTAMP + theta) - sin(theta)) + x;
		predictedY = (velocity/phi) * (-cos(phi*TIMESTAMP + theta) + cos(theta)) + y;
		predictedTheta = phi*TIMESTAMP + theta;
		predictedVel = velocity;
		predictedPhi = phi;
	}
	else
	{
		predictedX = x + velocity * TIMESTAMP * cos(theta);
		predictedY = y + velocity * TIMESTAMP * sin(theta);
		predictedTheta = theta;
		predictedVel = velocity;
		predictedPhi = 0.00001;
	}
	state.put(0,0, predictedX);
	state.put(1,0, predictedY);
	state.put(2,0, predictedTheta);
	state.put(3,0, predictedVel);
	state.put(4,0, predictedPhi);
}
void PointCell::computeCovarianceF(Matrix<double> state, Matrix<double>& F)
{
	double theta = state.get(2,0);
	double velocity = state.get(3,0);
	double phi = state.get(4,0);

	double f12, f13, f14, f22, f23, f24;

	f12 = (velocity/phi) * (-cos(theta) + cos(TIMESTAMP*phi + theta));
	f13 = (1/phi) * (sin(phi*TIMESTAMP + theta) - sin(theta));
	f14 = (((TIMESTAMP*velocity)/phi) * cos(TIMESTAMP*phi + theta)) - ((velocity/(phi*phi)) * (sin(phi*TIMESTAMP + theta) - sin(theta)));

	f22 = (velocity/phi) * (sin(phi*TIMESTAMP + theta) - sin(theta));
	f23 = (1/phi) * (-cos(phi*TIMESTAMP + theta) + cos(theta));
	f24 = (((TIMESTAMP*velocity)/phi) * sin(TIMESTAMP*phi + theta)) - ((velocity/(phi*phi)) * (-cos(phi*TIMESTAMP + theta) + cos(theta)));

	F.put(0,2, f12);
	F.put(0,3, f13);
	F.put(0,4, f14);
	F.put(1,2, f22);
	F.put(1,3, f23);
	F.put(1,4, f24);
}
