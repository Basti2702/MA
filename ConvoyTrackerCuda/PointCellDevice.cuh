/*
 * PointCellDevice.cuh
 *
 *  Created on: 07.10.2016
 *      Author: basti
 */

#ifndef POINTCELLDEVICE_CUH_
#define POINTCELLDEVICE_CUH_

#include "data.cuh"
#include <math.h>
#include <stdio.h>
#include <algorithm>
#include "Matrix.cuh"
#include <cuda.h>


class PointCellDevice {
public:
	PointCellDevice();
	virtual ~PointCellDevice();

	/*
	 * data = [stateVector | stateCopy | F | P | H | R | K | I | Q | S | tmp | tmp2]
	 */
	double data[260];

	double subInvtl;

	__host__ __device__ void predict();
	void update(double* newState);
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
	__host__ __device__ void initializeMemory();

private:

	__host__ __device__ void computeF();
	__host__ __device__ void computeCovarianceF();
	__host__ __device__ void writeP(int row, int col, double value);
	__host__ __device__ void writeQ(int row, int col, double value);
	__host__ __device__ void writeR(int row, int col, double value);
	__host__ __device__ void writeH(int row, int col, double value);
	__host__ __device__ void writeK(int row, int col, double value);
	__host__ __device__ void writeI(int row, int col, double value);
	__host__ __device__ void writeF(int row, int col, double value);
	__host__ __device__ void writeS(int row, int col, double value);
	__host__ __device__ void writeTmp(int row, int col, double value);
	__host__ __device__ void writeTmp2(int row, int col, double value);

	__host__ __device__ double getP(int row, int col);
	__host__ __device__ double getQ(int row, int col);
	__host__ __device__ double getR(int row, int col);
	__host__ __device__ double getH(int row, int col);
	__host__ __device__ double getK(int row, int col);
	__host__ __device__ double getI(int row, int col);
	__host__ __device__ double getF(int row, int col);
	__host__ __device__ double getS(int row, int col);
	__host__ __device__ double getTmp(int row, int col);
	__host__ __device__ double getTmp2(int row, int col);

	int ID;
};

#endif /* POINTCELLDEVICE_CUH_ */
