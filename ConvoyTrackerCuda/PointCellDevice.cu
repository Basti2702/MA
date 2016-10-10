#include "PointCellDevice.cuh"

/*
 * data* = [stateVector | stateCopy | F | P | H | R | K | I | Q | S | tmp | tmp2]
 */
PointCellDevice::PointCellDevice()
{
//	data = (double*) malloc(260*sizeof(double));
	initializeMemory();
	subInvtl = 0.5;
}

PointCellDevice::~PointCellDevice()
{
//	free(data);
}

__host__ __device__ void PointCellDevice::initializeMemory()
{
	//initialize data to 0
	for(int i=0; i<260; i++)
	{
		data[i] = 0;
	}

	for(int i=0; i<5; i++)
	{
		//P
		data[35 + i*5 + i] = 1000;
		//F
		data[10 + i*5 + i] = 1;
		//I
		data[135 + i*5 + i] = 1;
		//H
		data[60 + i*5 + i] = 1;
	}
	//F(2,4)
	data[10 + 2*5 +4] = TIMESTAMP;

	//Q
	data[160] = 0.000006;
	data[160 + 1*5 + 1] = 0.000006;
	data[160 + 2*5 + 2] = 0.0004;
	data[160 + 3*5 + 3] = 0.03097;
	data[160 + 4*5 + 4] = 0.0004;

	//R
	data[85] = 0.36;
	data[85 + 1*5 + 1] = 0.36;
	data[85 + 2*5 + 2] = 0.5;
	data[85 + 3*5 + 3] = 0.1;
	data[85 + 4*5 + 4] = 0.1;
}

__host__ __device__ void PointCellDevice::predict()
{
	//store copy of stateVector
	for(int i=0; i<5; i++)
	{
		data[i+5] = data[i];
	}
	computeF();
	computeCovarianceF();

	double tmp = 0;
	// Tmp = F*P
	for(int i=0; i<5; i++)
	{
		for(int j=0; j<5; j++)
		{
			tmp = 0;
			for(int k=0; k<5; k++)
			{
				tmp += getF(i,k)*getP(k,j);
			}
			writeTmp(i,j, tmp);
		}
	}

	//P = Tmp*F_t
	for(int i=0; i<5; i++)
	{
		for(int j=0; j<5; j++)
		{
			tmp = 0;
			for(int k=0; k<5; k++)
			{
				tmp += getTmp(i,k)*getF(j,k);
			}
			writeP(i,j, tmp);
		}
	}

	//P = P+Q
	for(int i=0; i<5; i++)
	{
		for(int j=0; j<5; j++)
		{
			tmp = getP(i,j) + getQ(i,j);
			writeP(i,j, tmp);
		}
	}
}

__host__ __device__ void PointCellDevice::computeF()
{
	double x = getX();
	double y = getY();
	double theta = getTheta();
	double velocity = getVelocity();
	double phi = getPhi();

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

	setX(predictedX);
	setY(predictedY);
	setTheta(predictedTheta);
	setVelocity(predictedVel);
	setPhi(predictedPhi);
}

__host__ __device__ void PointCellDevice::computeCovarianceF()
{
	double theta = getTheta();
	double velocity = getVelocity();
	double phi = getPhi();

	double f12, f13, f14, f22, f23, f24;

	f12 = (velocity/phi) * (-cos(theta) + cos(TIMESTAMP*phi + theta));
	f13 = (1/phi) * (sin(phi*TIMESTAMP + theta) - sin(theta));
	f14 = (((TIMESTAMP*velocity)/phi) * cos(TIMESTAMP*phi + theta)) - ((velocity/(phi*phi)) * (sin(phi*TIMESTAMP + theta) - sin(theta)));

	f22 = (velocity/phi) * (sin(phi*TIMESTAMP + theta) - sin(theta));
	f23 = (1/phi) * (-cos(phi*TIMESTAMP + theta) + cos(theta));
	f24 = (((TIMESTAMP*velocity)/phi) * sin(TIMESTAMP*phi + theta)) - ((velocity/(phi*phi)) * (-cos(phi*TIMESTAMP + theta) + cos(theta)));

	writeF(0,2,f12);
	writeF(0,3,f13);
	writeF(0,4,f14);
	writeF(1,2,f22);
	writeF(1,3,f23);
	writeF(1,4,f24);
}
void PointCellDevice::update(double* newState)
{
	double velocity, phi;
	double xNew = newState[0];
	double yNew = newState[1];
	double thetaNew = newState[2];

	double x = data[5];
	double y = data[6];
	double theta = data[7];
	velocity = sqrt((xNew - x) * (xNew - x) + (yNew - y)*(yNew - y)) / TIMESTAMP;
	phi = (thetaNew-theta) / TIMESTAMP;

	setVelocity(velocity);
	setPhi(phi);
	double tmp = 0;

	//tmp = H*P
	for(int i=0; i<5; i++)
	{
		for(int j=0; j<5; j++)
		{
			tmp = 0;
			for(int k=0; k<5; k++)
			{
				tmp += getH(i,k)*getP(k,j);
			}
			writeTmp(i,j, tmp);
		}
	}

	//S = tmp*H_t
	for(int i=0; i<5; i++)
	{
		for(int j=0; j<5; j++)
		{
			tmp = 0;
			for(int k=0; k<5; k++)
			{
				tmp += getTmp(i,k)*getH(j,k);
			}
			writeS(i,j, tmp);
		}
	}

	//S = S+R
	for(int i=0; i<5; i++)
	{
		for(int j=0; j<5; j++)
		{
			tmp = getS(i,j) + getR(i,j);
			writeS(i,j, tmp);
		}
	}

	//tmp = P*H_t
	for(int i=0; i<5; i++)
	{
		for(int j=0; j<5; j++)
		{
			tmp = 0;
			for(int k=0; k<5; k++)
			{
				tmp += getP(i,k)*getH(j,k);
			}
			writeTmp(i,j, tmp);
		}
	}

	//S inverse
	Matrix<double> S(5,5);
	for(int i=0; i<5;i++)
	{
		for(int j=0; j<5;j++)
		{
			S.put(i,j,getS(i,j));
		}
	}

	S.invert();
	for(int i=0; i<5;i++)
	{
		for(int j=0; j<5;j++)
		{
			writeS(i,j,S.get(i,j));
		}
	}

	//K = tmp*S_i
	for(int i=0; i<5; i++)
	{
		for(int j=0; j<5; j++)
		{
			tmp = 0;
			for(int k=0; k<5; k++)
			{
				tmp += getTmp(i,k)*getS(k,j);
			}
			writeK(i,j, tmp);
		}
	}

	//tmp = K*(newState-stateVector)
	for(int i=0; i<5; i++)
	{
		for(int j=0; j<1; j++)
		{
			tmp = 0;
//			for(int k=0; k<5; k++)
//			{
			tmp += getK(i,0)*(xNew-getX());
			tmp += getK(i,1)*(yNew-getY());
			tmp += getK(i,2)*(thetaNew-getTheta());
			tmp += getK(i,3)*(velocity-getVelocity());
			tmp += getK(i,4)*(phi-getPhi());
	//		}
			writeTmp(i,j, tmp);
		}
	}

	//stateVector = stateVector + tmp
	setX(getX() + getTmp(0,0));
	setY(getY() + getTmp(1,0));
	setTheta(getTheta() + getTmp(2,0));
	setVelocity(getVelocity() + getTmp(3,0));
	setPhi(getPhi() + getTmp(4,0));

	//tmp = K*H
	for(int i=0; i<5; i++)
	{
		for(int j=0; j<5; j++)
		{
			tmp = 0;
			for(int k=0; k<5; k++)
			{
				tmp += getK(i,k)*getH(k,j);
			}
			writeTmp(i,j, tmp);
		}
	}

	//tmp = I - tmp
	for(int i=0; i<5; i++)
	{
		for(int j=0; j<5; j++)
		{
			tmp = getI(i,j) - getTmp(i,j);
			writeTmp(i,j, tmp);
		}
	}

	//tmp2 = tmp*P
	for(int i=0; i<5; i++)
	{
		for(int j=0; j<5; j++)
		{
			tmp = 0;
			for(int k=0; k<5; k++)
			{
				tmp += getTmp(i,k)*getP(k,j);
			}
			writeTmp2(i,j, tmp);
		}
	}

	for(int i=0; i<5;i++)
	{
		for(int j=0; j<5; j++)
		{
			writeP(i,j, getTmp2(i,j));
		}
	}

}
int PointCellDevice::getID()
{
	return ID;
}
void PointCellDevice::setID(int id)
{
	ID = id;
}
__host__ __device__ double PointCellDevice::getX()
{
	return data[0];
}
__host__ __device__ double PointCellDevice::getY()
{
	return data[1];
}
__host__ __device__ double PointCellDevice::getTheta()
{
	return data[2];
}
__host__ __device__ double PointCellDevice::getVelocity()
{
	return data[3];
}
__host__ __device__ double PointCellDevice::getPhi()
{
	return data[4];
}

__host__ __device__ void PointCellDevice::setX(double x)
{
	data[0] = x;
}
__host__ __device__ void PointCellDevice::setY(double y)
{
	data[1] = y;
}
__host__ __device__ void PointCellDevice::setTheta(double theta)
{
	data[2] = theta;
}
__host__ __device__ void PointCellDevice::setVelocity(double velocity)
{
	data[3] = velocity;
}
__host__ __device__ void PointCellDevice::setPhi(double phi)
{
	data[4] = phi;
}

__host__ __device__ void PointCellDevice::writeP(int row, int col, double value)
{
	data[35 + row*5 + col] = value;
}

__host__ __device__ void PointCellDevice::writeF(int row, int col, double value)
{
	data[10 + row*5 + col] = value;
}

__host__ __device__ void PointCellDevice::writeH(int row, int col, double value)
{
	data[60 + row*5 + col] = value;
}

__host__ __device__ void PointCellDevice::writeR(int row, int col, double value)
{
	data[85 + row*5 + col] = value;
}

__host__ __device__ void PointCellDevice::writeK(int row, int col, double value)
{
	data[110 + row*5 + col] = value;
}

__host__ __device__ void PointCellDevice::writeI(int row, int col, double value)
{
	data[135 + row*5 + col] = value;
}

__host__ __device__ void PointCellDevice::writeQ(int row, int col, double value)
{
	data[160 + row*5 + col] = value;
}

__host__ __device__ void PointCellDevice::writeS(int row, int col, double value)
{
	data[185 + row*5 + col] = value;
}

__host__ __device__ void PointCellDevice::writeTmp(int row, int col, double value)
{
	data[210 + row*5 + col] = value;
}

__host__ __device__ void PointCellDevice::writeTmp2(int row, int col, double value)
{
	data[235 + row*5 + col] = value;
}

__host__ __device__ double PointCellDevice::getP(int row, int col)
{
	return data[35 + row*5 + col];
}

__host__ __device__ double PointCellDevice::getF(int row, int col)
{
	return data[10 + row*5 + col];
}

__host__ __device__ double PointCellDevice::getH(int row, int col)
{
	return data[60 + row*5 + col];
}

__host__ __device__ double PointCellDevice::getR(int row, int col)
{
	return data[85 + row*5 + col];
}

__host__ __device__ double PointCellDevice::getK(int row, int col)
{
	return data[110 + row*5 + col];
}

__host__ __device__ double PointCellDevice::getI(int row, int col)
{
	return data[135 + row*5 + col];
}

__host__ __device__ double PointCellDevice::getQ(int row, int col)
{
	return data[160 + row*5 + col];
}

__host__ __device__ double PointCellDevice::getS(int row, int col)
{
	return data[185 + row*5 + col];
}

__host__ __device__ double PointCellDevice::getTmp(int row, int col)
{
	return data[210 + row*5 + col];
}

__host__ __device__ double PointCellDevice::getTmp2(int row, int col)
{
	return data[235 + row*5 + col];
}
