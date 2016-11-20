/*
 * ConvoyTracker.cu
 *
 *  Created on: 06.06.2016
 *      Author: Sebastian Reinhart
 */

#include "ConvoyTracker.cuh"
#include <assert.h>
ConvoyTracker::ConvoyTracker() {
	currentSpeed = 0;
	currentYawRate = 0;
	x = 0;
	y = 0;
	yaw = 0;
	xOld = 0;
	yOld = 0;
	yawOld = 0;
	ID = 0;
	convoyID = 0;
	currentHistoryOnDevice = false;
	currentConvoyOnDevice = false;
	convoySize = 0;
	startIndexConvoys = 0;
	endIndexConvoys = 0;
	historySize = 0;
	startIndexHistory = 0;
	endIndexHistory = 0;
	convoyCheckSize = 0;
	intervalSize = 0;

	cudaError_t error;

	cudaStreamCreate(&stream2);
	cudaStreamCreate(&stream3);
	cudaStreamCreate(&stream4);
	cudaStreamCreate(&stream5);

	error = cudaHostAlloc((void**) &history, NUM_HIST*sizeof(History), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_history, NUM_HIST*sizeof(History));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_historyMatch, MAX_SEGMENTS*sizeof(int), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_historyMatch, MAX_SEGMENTS*sizeof(int));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_historyMatchSelf, sizeof(int), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_historyMatchSelf, sizeof(int));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_intervalMap, MAX_SEGMENTS*sizeof(PointCellDevice), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_intervalMap, MAX_SEGMENTS*sizeof(PointCellDevice));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_convoyCheck, MAX_SEGMENTS*sizeof(PointCellDevice), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_newVeh, MAX_SEGMENTS*sizeof(PointCellDevice));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_IDincluded, NUM_HIST*2*sizeof(int), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_IDincluded, NUM_HIST*2*sizeof(int));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_duplicate, NUM_HIST*sizeof(bool), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_duplicate, NUM_HIST*sizeof(bool));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_updateData, MAX_SEGMENTS*3*sizeof(double), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_updataData, MAX_SEGMENTS*3*sizeof(double));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_intvlIndex, MAX_SEGMENTS*sizeof(int), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_intvlIndex, MAX_SEGMENTS*sizeof(int));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_vehicles, MAX_SEGMENTS*sizeof(PointCellDevice), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_vehicles, MAX_SEGMENTS*sizeof(PointCellDevice));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_distance, MAX_SEGMENTS*MAX_SEGMENTS*sizeof(double), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_distance, MAX_SEGMENTS*MAX_SEGMENTS*sizeof(double));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	size_t sizeConv = NUM_CONV;
	sizeConv *= sizeof(Convoy);
	error = cudaHostAlloc((void **) &convoys, sizeConv, cudaHostAllocDefault);
	if(error != cudaSuccess)
	{
		printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_convoys, NUM_CONV*sizeof(Convoy));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &xSubInterval, sizeof(double), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	*xSubInterval = 0;

	error = cudaHostGetDevicePointer(&d_subIntvl_ptr, xSubInterval, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}
}

ConvoyTracker::~ConvoyTracker() {
	// TODO Auto-generated destructor stub
	cudaFreeHost(xSubInterval);
	cudaFreeHost(convoys);
	cudaFreeHost(history);
	cudaFreeHost(h_historyMatch);
	cudaFreeHost(h_convoyCheck);
	cudaFreeHost(h_intervalMap);
	cudaFreeHost(h_historyMatchSelf);
	cudaFreeHost(h_IDincluded);
	cudaFreeHost(h_vehicles);
	cudaFreeHost(h_distance);
	cudaFreeHost(h_duplicate);
	cudaFreeHost(h_intvlIndex);
	cudaFree(d_convoys);
	cudaFree(d_history);
	cudaFree(d_historyMatch);
	cudaFree(d_newVeh);
	cudaFree(d_intervalMap);
	cudaFree(d_historyMatchSelf);
	cudaFree(d_IDincluded);
	cudaFree(d_vehicles);
	cudaFree(d_distance);
	cudaFree(d_duplicate);
	cudaFree(d_intvlIndex);
	cudaStreamDestroy(stream2);
	cudaStreamDestroy(stream3);
	cudaStreamDestroy(stream4);
	cudaStreamDestroy(stream5);
}


std::string getNextMeasureAsString(int i)
{
	std::ostringstream number;
	if(i<10)
	{
		number << "000" << i;
	}
	else if(i<100)
	{
		number << "00" << i;
	}
	else if(i<1000)
	{
		number << "0" << i;
	}
	else
	{
		number << i;
	}
	return number.str();
}

__device__ void shiftRotateHistory(History* d_pc, double x, double y, double theta, int index)
{
	//update history
	if(((index < d_pc->endIndex)  && (d_pc->endIndex > d_pc->startIndex)) || ((d_pc->endIndex < d_pc->startIndex) && (index != d_pc->endIndex)))
	{

		d_pc->tracks[index].subIntvl += x;
		int numIntervals = (int) ((d_pc->tracks[index].subIntvl) / INTERVALL_LENGTH);
		d_pc->tracks[index].x -= numIntervals;
		d_pc->tracks[index].subIntvl -= numIntervals;

		double angleInRadians = theta*M_PI/180.0;
		double mat[2][2] = { { cos(angleInRadians), -sin(angleInRadians) },
				{ sin(angleInRadians), cos(angleInRadians) } };

		//update history
		d_pc->tracks[index].y -= y;
		d_pc->tracks[index].theta -= angleInRadians;

		double xAbs = d_pc->tracks[index].x;
		double yAbs = d_pc->tracks[index].y;

		xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
		yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

		d_pc->tracks[index].y -= yAbs;
		d_pc->tracks[index].subIntvl -= xAbs;
	}
}

__device__ void shiftRotateConvoy(Convoy* d_eml, double x, double y, double theta, int index)
{
	if(((index < d_eml->endIndexTracks)  && (d_eml->endIndexTracks > d_eml->startIndexTracks)) || ((d_eml->endIndexTracks < d_eml->startIndexTracks) && (index != d_eml->endIndexTracks)))
	{
		d_eml->tracks[index].subIntvl += x;
		int numIntervals = (int) ((d_eml->tracks[index].subIntvl) / INTERVALL_LENGTH);
		d_eml->tracks[index].x -= numIntervals;
		d_eml->tracks[index].subIntvl -= numIntervals;

		double angleInRadians = theta*M_PI/180.0;
		double mat[2][2] = { { cos(angleInRadians), -sin(angleInRadians) },
				{ sin(angleInRadians), cos(angleInRadians) } };

		d_eml->tracks[index].y -= y;
		d_eml->tracks[index].theta -= angleInRadians;

		double xAbs = d_eml->tracks[index].x;
		double yAbs = d_eml->tracks[index].y;

		xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
		yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

		d_eml->tracks[index].y -= yAbs;
		d_eml->tracks[index].subIntvl -= xAbs;
	}
}

__device__ void computeIntervalMap(PointCellDevice* d_interval, double xMotion, double yMotion, double angle, double* xSubInterval)
{
	double angleInRadians = angle * M_PI / 180;
	*xSubInterval += xMotion;
	int numIntervals = (int) (*xSubInterval / INTERVALL_LENGTH);
	*xSubInterval -= numIntervals;
	for (int i = 0; i < numIntervals; i++)
	{
		double x = d_interval->getX();
		int interval = floor(x) + CARINTERVAL;
		if(interval == 0)
		{
			//delete content
			d_interval->setX(-10000);
			continue;
		}
		d_interval->setX(floor(x) - 0.5);
	}
	int	interval = floor(d_interval->getX());
	//1.Step correct directions of stored PCs
	d_interval->setY(d_interval->getY() - yMotion);
	d_interval->setTheta(d_interval->getTheta() - angleInRadians);

	//2. compensate rotation
	double xAbs = ( interval - CARINTERVAL + 0.5) * INTERVALL_LENGTH
			- *xSubInterval;
	double yAbs = d_interval->getY();


	double mat[2][2] = { { cos(angleInRadians), -sin(angleInRadians) },
			{ sin(angleInRadians), cos(angleInRadians) } };
	xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
	yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

	d_interval->setY(d_interval->getY() - yAbs);

	if(xAbs > 0.5*INTERVALL_LENGTH)
	{
		d_interval->setX(interval + 1.5);
	//	currentVehicle.subInvtl = xAbs - 0.5;
	}
	else if(xAbs < -0.5*INTERVALL_LENGTH)
	{
		d_interval->setX(interval - 0.5);
	//	currentVehicle.subInvtl = xAbs + 0.5;
	}
	else
	{
		d_interval->subInvtl -= xAbs;
	}

//	d_interval->predict();
}

__device__ bool findHistoryMatch(PointCellDevice* trackedVehicles, History* d_history, int index)
{
	bool result = (d_history->ID != trackedVehicles->getID());
	result = (result && (d_history->tracks[index].x - 0.5 <= trackedVehicles->getX()));
	result = (result && (trackedVehicles->getX() <= d_history->tracks[index].x + 0.5));
	result = (result && (d_history->tracks[index].y - 1.0 <= trackedVehicles->getY()));
	result = (result && (trackedVehicles->getY() <= d_history->tracks[index].y + 1.0));
//	result = (result && (((index < d_history->endIndex)  && (d_history->endIndex > d_history->startIndex)) || ((d_history->endIndex < d_history->startIndex) && (index != d_history->endIndex))));
	//result = (result && (index >= d_history->startIndex));
	return result;
}

__device__ bool findHistoryMatchSelf(History* d_history, int index)
{
	bool result = true;
	result = (result && (d_history->tracks[index].x - 0.5 <= 0));
	result = (result && (0 <= d_history->tracks[index].x + 0.5));
	result = (result && (d_history->tracks[index].y - 1.0 <= 0));
	result = (result && (0 <= d_history->tracks[index].y + 1.0));
//	result = (result && (((index < d_history->endIndex)  && (d_history->endIndex > d_history->startIndex)) || ((d_history->endIndex < d_history->startIndex) && (index != d_history->endIndex))));
	//result = (result && (index >= d_history->startIndex));
	return result;
}

__device__ void predictDevice(PointCellDevice* vehicle, int index)
{
	int state = index%5;
	//row
	int i = index / 5 ;
	//column
	int j = state;
	vehicle->data[state+5] = vehicle->data[state];
	__syncthreads();
	vehicle->computeF();
	vehicle->computeCovarianceF();
	double tmp = 0;
	//Tmp = F*P
	for(int k=0; k<5; k++)
	{
		tmp += vehicle->getF(i,k)*vehicle->getP(k,j);
	}
	vehicle->writeTmp(i,j, tmp);
	__syncthreads();
	//P = tmp*F_t
	tmp = 0;
	for(int k=0; k<5; k++)
	{
		tmp += vehicle->getTmp(i,k)*vehicle->getF(j,k);
	}
	vehicle->writeP(i,j, tmp);
	__syncthreads();
	//P = P+Q
	tmp = vehicle->getP(i,j) + vehicle->getQ(i,j);
	vehicle->writeP(i,j, tmp);
}

__global__ void compensateEgoMotionMap(PointCellDevice* d_interval, double* d_subIntvl, double x, double y, double angle)
{
	computeIntervalMap(&(d_interval[threadIdx.x]), x, y, angle, d_subIntvl);
}
__global__ void compensateEgoMotionHistory(History* d_history, double x, double y, double angle)
{
	shiftRotateHistory(&(d_history[blockIdx.x]), x, y, angle, threadIdx.x);
}

__global__ void compensateEgoMotionConvoy(Convoy* d_convoy, double x, double y, double angle)
{
	shiftRotateConvoy(&(d_convoy[blockIdx.x]), x, y, angle, threadIdx.x);
}

__global__ void findConvoyDevice(PointCellDevice* trackedVehicles, History* d_history, int* d_historyMatch)
{
	if(((threadIdx.x < d_history[blockIdx.x].endIndex)  && (d_history[blockIdx.x].endIndex > d_history[blockIdx.x].startIndex)) || ((d_history[blockIdx.x].endIndex < d_history[blockIdx.x].startIndex) && (threadIdx.x != d_history[blockIdx.x].endIndex)))
	{
		if(findHistoryMatch(&(trackedVehicles[blockIdx.y]),&(d_history[blockIdx.x]),threadIdx.x))
		{
			atomicMin(&(d_historyMatch[blockIdx.y]), d_history[blockIdx.x].ID);
		}
	}
}
__global__ void findConvoyDeviceSelf(History* d_history, int* d_historyMatchSelf)
{
	if(((threadIdx.x < d_history[blockIdx.x].endIndex)  && (d_history[blockIdx.x].endIndex > d_history[blockIdx.x].startIndex)) || ((d_history[blockIdx.x].endIndex < d_history[blockIdx.x].startIndex) && (threadIdx.x != d_history[blockIdx.x].endIndex)))
	{
		if(findHistoryMatchSelf(&(d_history[blockIdx.x]),threadIdx.x))
		{
			atomicMin(d_historyMatchSelf, d_history[blockIdx.x].ID);
		}
	}
}

__global__ void memSetHistoryMatch(int* d_historyMatch)
{
	d_historyMatch[threadIdx.x] = INT_MAX;
}

/*
 * Run Kalman-Filter Predict on Device with #vehicles as Blocks and 25 Threads per Block
 */
__global__ void predict(PointCellDevice* d_interval)
{
	predictDevice(&(d_interval[blockIdx.x]), threadIdx.x);
}

/*
 * Run Kalman-Filter Update on Device with 25 Threads
 */
__device__ void updateDevice(PointCellDevice* d_interval, int index, double velocity, double phi, double xNew, double yNew, double thetaNew)
{
	//row
	int i = index / 5;
	//column
	int j = index % 5;

	double tmp = 0;

	//tmp = H*P
	for(int k=0; k<5; k++)
	{
		tmp += d_interval->getH(i,k)*d_interval->getP(k,j);
	}
	d_interval->writeTmp(i,j, tmp);
	__syncthreads();
	//S = tmp*H_t
	tmp = 0;
	for(int k=0; k<5; k++)
	{
		tmp += d_interval->getTmp(i,k)*d_interval->getH(j,k);
	}
	d_interval->writeS(i,j, tmp);
	__syncthreads();
	//S = S+R
	tmp = d_interval->getS(i,j) + d_interval->getR(i,j);
	d_interval->writeS(i,j, tmp);
	__syncthreads();
	//tmp = P*H_t
	tmp = 0;
	for(int k=0; k<5; k++)
	{
		tmp += d_interval->getP(i,k)*d_interval->getH(j,k);
	}
	d_interval->writeTmp(i,j, tmp);
	__syncthreads();
	//invertS
	if(threadIdx.x == 0)
	{
		d_interval->invertS();
	}
	__syncthreads();
	//K = tmp*S_i
	tmp = 0;
	for(int k=0; k<5; k++)
	{
		tmp += d_interval->getTmp(i,k)*d_interval->getS(k,j);
	}
	d_interval->writeK(i,j, tmp);
	__syncthreads();
	//tmp = K*(newState-stateVector)
	tmp = 0;
	tmp += d_interval->getK(i,0)*(xNew-d_interval->getX());
	tmp += d_interval->getK(i,1)*(yNew-d_interval->getY());
	tmp += d_interval->getK(i,2)*(thetaNew-d_interval->getTheta());
	tmp += d_interval->getK(i,3)*(velocity-d_interval->getVelocity());
	tmp += d_interval->getK(i,4)*(phi-d_interval->getPhi());
	d_interval->writeTmp(i,j, tmp);
	__syncthreads();
	//stateVector = stateVector + tmp
	if(threadIdx.x == 0)
	{
	d_interval->setX(d_interval->getX() + d_interval->getTmp(0,0));
	d_interval->setY(d_interval->getY() + d_interval->getTmp(1,0));
	d_interval->setTheta(d_interval->getTheta() + d_interval->getTmp(2,0));
	d_interval->setVelocity(d_interval->getVelocity() + d_interval->getTmp(3,0));
	d_interval->setPhi(d_interval->getPhi() + d_interval->getTmp(4,0));
	}
	__syncthreads();
	//tmp = K*H
	tmp = 0;
	for(int k=0; k<5; k++)
	{
		tmp += d_interval->getK(i,k)*d_interval->getH(k,j);
	}
	d_interval->writeTmp(i,j, tmp);
	__syncthreads();
	//tmp = I- tmp
	tmp = d_interval->getI(i,j) - d_interval->getTmp(i,j);
	d_interval->writeTmp(i,j, tmp);
	__syncthreads();
	//tmp2 = tmp*P
	tmp = 0;
	for(int k=0; k<5; k++)
	{
		tmp += d_interval->getTmp(i,k)*d_interval->getP(k,j);
	}
	d_interval->writeTmp2(i,j, tmp);
	__syncthreads();
	d_interval->writeP(i,j, d_interval->getTmp2(i,j));
}
__global__ void updateOne(PointCellDevice* d_interval, int index, double velocity, double phi, double xNew, double yNew, double thetaNew)
{
	updateDevice(d_interval,threadIdx.x,velocity, phi,xNew,yNew,thetaNew);
}
__global__ void updateKernel(PointCellDevice* d_intvl, double* d_updateData, int* d_intvlIndex)
{

	int index = d_intvlIndex[blockIdx.x];
	double xNew = d_updateData[blockIdx.x*3];
	double yNew = d_updateData[blockIdx.x*3+1];
	double thetaNew = d_updateData[blockIdx.x*3+2];
/*	if(threadIdx.x == 0)
	{
		printf("Update ID %d with x %f y %f theta %f, updateValues x %f y %f theta %f\n", d_intvl[index].getID(),d_intvl[index].getX(),d_intvl[index].getY(),d_intvl[index].getTheta(),xNew, yNew,thetaNew);

	}*/
	double x = d_intvl[index].data[5];
	double y = d_intvl[index].data[6];
	double theta = d_intvl[index].data[7];
	double velocity = sqrt((xNew - x) * (xNew - x) + (yNew - y)*(yNew - y)) / TIMESTAMP;
	double phi = (thetaNew-theta) / TIMESTAMP;
	if(threadIdx.x == 0)
	{
		d_intvl[index].setVelocity(velocity);
		d_intvl[index].setPhi(phi);
	}
	updateDevice(&(d_intvl[index]),threadIdx.x,velocity, phi,xNew,yNew,thetaNew);
}
__global__ void findIDInConvoyDevice(Convoy* d_convoy, int* d_IDIncluded, int id1, int id2)
{
	if(((threadIdx.x < d_convoy[blockIdx.x].endIndexID)  && (d_convoy[blockIdx.x].endIndexID > d_convoy[blockIdx.x].startIndexID)) || ((d_convoy[blockIdx.x].endIndexID < d_convoy[blockIdx.x].startIndexID) && (threadIdx.x != d_convoy[blockIdx.x].endIndexID)))
	{
		int index = blockIdx.x*2;
		d_IDIncluded[index] = INT_MAX;
		d_IDIncluded[index+1] = INT_MAX;
		__syncthreads();
		int result = (d_convoy[blockIdx.x].participatingVehicles[threadIdx.x] == id1);
		if(result)
		{
			atomicMin(&(d_IDIncluded[index]), threadIdx.x);
		}
		result = (d_convoy[blockIdx.x].participatingVehicles[threadIdx.x] == id2);
		if(result)
		{
			atomicMin(&(d_IDIncluded[index+1]), threadIdx.x);
		}
	}
}

__global__ void checkConvoyForDuplicateDevice(Convoy* d_convoy, PointCellDevice* d_vehicle, bool* d_duplicate)
{
	if(((threadIdx.x < d_convoy[blockIdx.x].endIndexTracks)  && (d_convoy[blockIdx.x].endIndexTracks > d_convoy[blockIdx.x].startIndexTracks)) || ((d_convoy[blockIdx.x].endIndexTracks < d_convoy[blockIdx.x].startIndexTracks) && (threadIdx.x != d_convoy[blockIdx.x].endIndexTracks)))
	{
		d_duplicate[blockIdx.x] = true;
		__syncthreads();
		bool result = (d_convoy[blockIdx.x].tracks[threadIdx.x].x != (floor(d_vehicle->getX())+0.5));
		if(!result)
		{
			d_duplicate[blockIdx.x] = d_duplicate[blockIdx.x] && result;
		}
	}
}
__global__ void checkHistoryForDuplicateDevice(History* d_history, PointCellDevice* d_intvl, int* d_intvlIndex, int* d_IDincluded, bool* d_duplicate)
{
	//printf("HistoryIndex block %d thread %d: %d\n",blockIdx.x, threadIdx.x, d_IDincluded[blockIdx.x]);
	if(((threadIdx.x < d_history[d_IDincluded[blockIdx.x]].endIndex)  && (d_history[d_IDincluded[blockIdx.x]].endIndex > d_history[d_IDincluded[blockIdx.x]].startIndex)) || ((d_history[d_IDincluded[blockIdx.x]].endIndex < d_history[d_IDincluded[blockIdx.x]].startIndex) && (threadIdx.x != d_history[d_IDincluded[blockIdx.x]].endIndex)))
	{
		d_duplicate[blockIdx.x] = true;
		int index = d_intvlIndex[blockIdx.x];
		int intvl = floor(d_intvl[index].getX());
		intvl += 0.5;
		if(d_history[d_IDincluded[blockIdx.x]].tracks[threadIdx.x].x == intvl)
		{
			d_duplicate[blockIdx.x] = false;
		}
	}
}
__global__ void findHistoryWithIDDevice(History* d_history, PointCellDevice* d_intvl, int* d_intvlIndex, int* d_IDincluded)
{
	int index = d_intvlIndex[threadIdx.x];
	int ID = d_intvl[index].getID();
	if(d_history[blockIdx.x].ID == ID)
	{
		d_IDincluded[threadIdx.x] = blockIdx.x;
	}
}

__global__ void addUpdatedPositionToHistoryDevice(History* d_history, PointCellDevice* d_intvl, int* d_intvlIndex, int* d_IDincluded, bool* d_duplicate)
{
	int intvl = floor(d_intvl[d_intvlIndex[threadIdx.x]].getX());
//		printf("Intervall ID %d after update: %d\n",h_intervalMap[h_intvlIndex[i]].getID(), intvl);
	d_intvl[d_intvlIndex[threadIdx.x]].setX(intvl+ 0.5);
//		int historyIndex = findHistoryWithID(h_intervalMap[h_intvlIndex[i]].getID());
	int historyIndex = d_IDincluded[threadIdx.x];
//		printf("historyIndex: %d\n", historyIndex);
	if(d_duplicate[threadIdx.x]/*checkHistoryForDuplicate(intvl + 0.5, historyIndex)*/)
	{
		int index = d_history[historyIndex].endIndex;
		d_history[historyIndex].tracks[index].subIntvl = 0.5;
		d_history[historyIndex].tracks[index].x = d_intvl[d_intvlIndex[threadIdx.x]].getX();
		d_history[historyIndex].tracks[index].y = d_intvl[d_intvlIndex[threadIdx.x]].getY();
		d_history[historyIndex].tracks[index].theta = d_intvl[d_intvlIndex[threadIdx.x]].getTheta();
		index = (index+1)%MAX_LENGTH_HIST_CONV;
		d_history[historyIndex].endIndex = index;
		if(index == d_history[historyIndex].startIndex)
		{
			d_history[historyIndex].startIndex = (d_history[historyIndex].startIndex+1)%NUM_HIST;
		}
	}
}

__global__ void computeDistancesDevice(PointCellDevice* d_vehicles, PointCellDevice* d_intvl, double* d_distance)
{
	int index = blockIdx.x*MAX_SEGMENTS + threadIdx.x;
	double x = d_vehicles[blockIdx.x].getX();
	double y = d_vehicles[blockIdx.x].getY();
	double theta = d_vehicles[blockIdx.x].getTheta();
	double x1 = d_intvl[threadIdx.x].getX();
	double y1 = d_intvl[threadIdx.x].getY();
	double theta1 = d_intvl[threadIdx.x].getTheta();

	d_distance[index] = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1) + (theta - theta1)*(theta - theta1));
}

int main()
{
#ifdef CREATE_MEASURES
	PGMReader pgmReader;
	double speed = 4.0/3.0;
	for(int i=0; i<NUM_MEASUREMENT; i++)
	{
		std::string number = getNextMeasureAsString(i);
		pgmReader.simulateLaserRays(number);

	/*	std::ofstream EMLMeasureFile;
		std::ostringstream measurePath;
		measurePath << "./Laserdata/EML" << number << ".txt";
		EMLMeasureFile.open (measurePath.str().c_str());
		EMLMeasureFile << ((double)i)*speed << " 0 0 120 0" << std::endl;
		EMLMeasureFile.close();*/
	}
#endif

	int devID = 0;

	cudaError_t error;
	cudaDeviceProp deviceProp;
	error = cudaGetDevice(&devID);

	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaGetDeviceProperties(&deviceProp, devID);

	if (deviceProp.computeMode == cudaComputeModeProhibited) {
		fprintf(stderr,
				"Error: device is running in <Compute Mode Prohibited>, no threads can use ::cudaSetDevice().\n");
		exit(EXIT_SUCCESS);
	}

	 if (deviceProp.canMapHostMemory != 1){
	  fprintf(stderr, "Device cannot map memory!\n");
	  return 1;
	}

	if (error != cudaSuccess) {
		printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	} else {
#ifdef PRINT
		printf("GPU Device %d: \"%s\" with compute capability %d.%d\n\n", devID,
				deviceProp.name, deviceProp.major, deviceProp.minor);
#endif
	}

	cudaSetDeviceFlags(cudaDeviceMapHost);

	cudaEvent_t startEvent, stopEvent, start2Event, stop2Event;

	cudaEventCreate(&startEvent);
	cudaEventCreate(&stopEvent);
	cudaEventCreate(&start2Event);
	cudaEventCreate(&stop2Event);
	float time = 0;
	cudaEventRecord(startEvent, 0);
	ConvoyTracker tracker;
	cudaEventRecord(stopEvent, 0);
	cudaEventSynchronize(stopEvent);
	cudaEventElapsedTime(&time,startEvent,stopEvent);
#if SZENARIO == 6
	std::vector<PointCellDevice> vehiclesSim;
	error = cudaHostAlloc((void**) &tracker.h_vehicleSim, MAX_SEGMENTS*sizeof(PointCellDevice), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **)&tracker.d_vehicleSim, MAX_SEGMENTS*sizeof(PointCellDevice));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}
	for(uint i=0;i <20; i++)
	{
		tracker.h_vehicleSim[i].initializeMemory();
		if(i%2 == 0)
		{
			tracker.h_vehicleSim[i].setY(-3);
			tracker.h_vehicleSim[i].setVelocity(38.9);
		}
		else
		{
			tracker.h_vehicleSim[i].setY(3);
			tracker.h_vehicleSim[i].setVelocity(27.8);
		}
		tracker.h_vehicleSim[i].setX((i/2)*8);
		tracker.h_vehicleSim[i].setTheta(0);
		tracker.h_vehicleSim[i].setPhi(0);
	//	vehiclesSim.push_back(tmp);
	//	std::cout << "x: " << tmp.getX() << " y: " << tmp.getY() << " theta: " << tmp.getTheta() << " Vel: " << tmp.getVelocity() << " Phi: " << tmp.getPhi() << std::endl;
	}
	cudaMemcpy(tracker.d_vehicleSim, tracker.h_vehicleSim, MAX_SEGMENTS*sizeof(PointCellDevice), cudaMemcpyHostToDevice);
#endif
	cudaEventRecord(startEvent, 0);
	int vehicleCount = 0;
	float compensateHistory[NUM_MEASUREMENT];
	for(int i=0; i<NUM_MEASUREMENT; i++)
	{
		cudaEventRecord(start2Event, 0);
		std::vector<PointCellDevice*> trackedVehicles;
		std::string number = getNextMeasureAsString(i);
		tracker.readEMLData(number);

		//1. Compensate own vehicle motion
		double deltaX = tracker.getX() - tracker.getXOld();
		double deltaY = tracker.getY() - tracker.getYOld();
		double deltaYaw = tracker.getYaw() - tracker.getYawOld();
		double angleInRadians = deltaYaw * M_PI / 180;
		double mat[2][2] = { { cos(angleInRadians), -sin(angleInRadians) },
				{ sin(angleInRadians), cos(angleInRadians) } };

		tracker.transformDataToDevice();
			if(tracker.historySize > 0)
			{
				compensateEgoMotionHistory<<<tracker.historySize, MAX_LENGTH_HIST_CONV,0, tracker.stream2>>>(tracker.d_history, deltaX, deltaY, deltaYaw);
			}
			vehicleCount = tracker.reader.processLaserData(number,tracker.getCurrentSpeed(), tracker.getCurrentYawRate(), tracker.h_vehicles);
			if(tracker.convoySize > 0)
			{
				compensateEgoMotionConvoy<<<tracker.convoySize, MAX_LENGTH_HIST_CONV,0, tracker.stream3>>>(tracker.d_convoys, deltaX, deltaY, deltaYaw);
			}
			if(tracker.intervalSize > 0)
			{
//			tracker.shiftStructure(deltaX);
//				tracker.rotateStructure(deltaYaw, deltaY);

				compensateEgoMotionMap<<<1,tracker.intervalSize,0,tracker.stream4>>>(tracker.d_intervalMap, tracker.d_subIntvl_ptr, deltaX, deltaY, deltaYaw);
				predict<<<tracker.intervalSize,25,0,tracker.stream4>>>(tracker.d_intervalMap);

			}

		/*	tracker.shiftStructure(deltaX);
			tracker.rotateStructure(deltaYaw, deltaY);
			for(uint j=0; j<tracker.intervalSize;j++)
			{
				tracker.h_intervalMap[j].predict();
				trackedVehicles.push_back(&(tracker.h_intervalMap[j]));
			}*/


		/*if(tracker.intervalSize >0)
		{
			predict<<<tracker.intervalSize,25,0,stream3>>>(tracker.d_intervalMap_ptr);
		}*/
		tracker.transformDataFromDevice();
		for(uint k = 0; k < tracker.convoySize; k++)
		{
			tracker.convoys[k].highestValue.subIntvl += deltaX;
			int numIntervals = (int) ((tracker.convoys[k].highestValue.subIntvl) / INTERVALL_LENGTH);
			tracker.convoys[k].highestValue.x -= numIntervals;
			tracker.convoys[k].highestValue.subIntvl -= numIntervals;

			tracker.convoys[k].highestValue.y -= deltaY;
			tracker.convoys[k].highestValue.theta -= angleInRadians;

			double xAbs = tracker.convoys[k].highestValue.x;
			double yAbs = tracker.convoys[k].highestValue.y;

			xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
			yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

			tracker.convoys[k].highestValue.y -= yAbs;
			tracker.convoys[k].highestValue.subIntvl -= xAbs;
		}
		for(uint j=0; j<tracker.intervalSize;j++)
		{
		//	tracker.h_intervalMap[j].predict();
			trackedVehicles.push_back(&(tracker.h_intervalMap[j]));
		}
		//3. Associate and Update
#if SZENARIO == 6
		tracker.associateAndUpdate(20, trackedVehicles);
#else
		tracker.associateAndUpdate(vehicleCount, trackedVehicles);
#endif
		cudaEventRecord(stop2Event, 0);
		cudaEventSynchronize(stop2Event);
		 float time3;
		 cudaEventElapsedTime(&time3, start2Event, stop2Event);
		 compensateHistory[i] = time3;
	}
	cudaEventRecord(stopEvent, 0);
	cudaEventSynchronize(stopEvent);

	float sumH = 0;

	for(int i = 0; i< NUM_MEASUREMENT; i++)
	{
		sumH += compensateHistory[i];
	}
	sumH /= NUM_MEASUREMENT;
#ifdef PRINT
	std::cout << "Duration of compensate History: " << sumH << std::endl;
#endif
	 float time2;
	 cudaEventElapsedTime(&time2, startEvent, stopEvent);
#ifdef PRINT
	 std::cout << "Overall Time: " << time +time2<< std::endl;
#else
	 std::cout << time + time2 << std::endl;
#endif
#if SZENARIO == 6
	 cudaFreeHost(tracker.h_vehicleSim);
#endif

	tracker.visualizeConvoys();
	tracker.visualizeHistory();
	return 0;
}

/**
 * Stores current Speed and yaw rate from file to class variables
 */
void ConvoyTracker::readEMLData(std::string number)
{
	std::ostringstream measurePath;
	measurePath << EMLPATH << number << ".txt";
#ifdef PRINT
	std::cout << measurePath.str() << std::endl;
#endif
	std::ifstream input(measurePath.str().c_str());
    std::string line;
    std::string segment;

    if(std::getline( input, line ))
    {
    	std::stringstream ss;

    	ss << line;

    	int dataCnt = 1;

    	while(std::getline(ss, segment, ' '))
    	{
			switch (dataCnt)
			{
				case 1:
				{
					//x in m
					xOld = x;
					x = atof(segment.c_str());
					++dataCnt;
					break;
				}
				case 2:
				{
					//y in m
					yOld = y;
					y = atof(segment.c_str());
					++dataCnt;
					break;
				}
				case 3:
				{
					//yaw in °
					yawOld = yaw;
					yaw = atof(segment.c_str());
					++dataCnt;
					break;
				}
				case 4:
				{
					//velocity in kmh
					//Compute value in m/s
					currentSpeed = atof(segment.c_str()) / 3.6;
					++dataCnt;
					break;
				}
				case 5:
				{
					//yaw rate in °/s
					//Compute value in rad/s
					currentYawRate = atof(segment.c_str()) * M_PI / 180.0;
					break;
				}
			}
    	}
    	EMLPos curPos;
    	curPos.x = x;
    	curPos.y = y;
    	curPos.theta = yaw;
    	EML.push_back(curPos);
    }
}

double ConvoyTracker::getCurrentSpeed() const {
	return currentSpeed;
}

void ConvoyTracker::setCurrentSpeed(double currentSpeed) {
	this->currentSpeed = currentSpeed;
}

double ConvoyTracker::getCurrentYawRate() const {
	return currentYawRate;
}

void ConvoyTracker::setCurrentYawRate(double currentYawRate) {
	this->currentYawRate = currentYawRate;
}

double ConvoyTracker::getX() const {
	return x;
}

void ConvoyTracker::setX(double x) {
	this->x = x;
}

double ConvoyTracker::getXOld() const {
	return xOld;
}

void ConvoyTracker::setXOld(double old) {
	this->xOld = old;
}

double ConvoyTracker::getY() const {
	return y;
}

void ConvoyTracker::setY(double y) {
	this->y = y;
}

double ConvoyTracker::getYaw() const {
	return yaw;
}

void ConvoyTracker::setYaw(double yaw) {
	this->yaw = yaw;
}

double ConvoyTracker::getYawOld() const {
	return yawOld;
}

void ConvoyTracker::setYawOld(double yawOld) {
	this->yawOld = yawOld;
}

double ConvoyTracker::getYOld() const {
	return yOld;
}

void ConvoyTracker::setYOld(double old) {
	yOld = old;
}

/**
 * Searches for corresponding vehicles using Global Nearest Neighbor algorithm and updates the results
 */
void ConvoyTracker::associateAndUpdate(int vehicleCount, std::vector<PointCellDevice*> trackedVehicles)
{
	//initialize all IDs in possible history to -1 to have no false detection in findConvoy
	memSetHistoryMatch<<<1,MAX_SEGMENTS,0,stream2>>>(d_historyMatch);
/*#if SZENARIO == 6
 *  cudaMemcpy(d_vehicles, h_vehicles, vehicleCount*sizeof(PointCellDevice), cudaMemcpyHostToDevice);
	computeDistancesDevice<<<vehicleCount,intervalSize,0,stream3>>>(d_vehicleSim,d_intervalMap,d_distance);
	cudaMemcpy(h_distance, d_distance, vehicleCount*MAX_SEGMENTS*sizeof(double), cudaMemcpyDeviceToHost);
#else
	cudaMemcpy(d_vehicles, h_vehicles, vehicleCount*sizeof(PointCellDevice), cudaMemcpyHostToDevice);
	computeDistancesDevice<<<vehicleCount,intervalSize,0,stream3>>>(d_vehicles,d_intervalMap,d_distance);
	cudaMemcpy(h_distance, d_distance, vehicleCount*MAX_SEGMENTS*sizeof(double), cudaMemcpyDeviceToHost);
#endif*/
	convoyCheckSize = 0;
	int updateCounter = 0;
	int indexCounter = trackedVehicles.size();
	std::vector<int> indicesToAdd;
	std::vector<PointCellDevice*> updateCheck;
/*	for(int i=0; i<intervalSize; i++)
	{
		h_intvlIndex[i] = i;
	}
	cudaStreamSynchronize(stream3); */
	for(uint i = 0; i<vehicleCount; i++)
	{
#if SZENARIO == 6
		double x = h_vehicleSim[i].getX();
		double y = h_vehicleSim[i].getY();
		double theta = h_vehicleSim[i].getTheta();
#else
		double x = h_vehicles[i].getX();
		double y = h_vehicles[i].getY();
		double theta = h_vehicles[i].getTheta();
#endif
		double minDist = INT_MAX;
		int minIndex = INT_MAX;
#ifdef PRINT
		std::cout << "X: " << x << " Y: " << y << " Theta: " << theta <<std::endl;
#endif
		for(uint j = 0; j<trackedVehicles.size(); j++)
		{
			double x1 = trackedVehicles.at(j)->getX();
			double y1 = trackedVehicles.at(j)->getY();
			double theta1 = trackedVehicles.at(j)->getTheta();
#ifdef PRINT
			std::cout << "X1: " << x1 << " Y1: " << y1<< " Theta1: " << theta1 <<std::endl;
#endif
			double dist = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1) + (theta - theta1)*(theta - theta1));

		//	double dist = h_distance[i*MAX_SEGMENTS+j];
			if(dist < minDist)
			{
				minDist = dist;
				minIndex = j;
			}
		}
#ifdef PRINT
		std::cout << "Min distance: " << minDist << std::endl;
#endif
		if(minDist > ASSOCIATION_THRESHOLD)
		{
			//do not associate vehicles with to big distance in between
			//create new track instead
			++ID;
#if SZENARIO == 6
			h_vehicleSim[i].setID(ID);
#else
			h_vehicles[i].setID(ID);
#endif
			indicesToAdd.push_back(i);
			history[endIndexHistory].ID = ID;
			history[endIndexHistory].tracks[0].subIntvl = 0.5;
#if SZENARIO == 6
			history[endIndexHistory].tracks[0].x = h_vehicleSim[i].getX();
			history[endIndexHistory].tracks[0].y = h_vehicleSim[i].getY();
			history[endIndexHistory].tracks[0].theta = h_vehicleSim[i].getTheta();
#else
			history[endIndexHistory].tracks[0].x = h_vehicles[i].getX();
			history[endIndexHistory].tracks[0].y = h_vehicles[i].getY();
			history[endIndexHistory].tracks[0].theta = h_vehicles[i].getTheta();
#endif
			history[endIndexHistory].startIndex = 0;
			history[endIndexHistory].endIndex = 1;
			int index = (endIndexHistory+1)%NUM_HIST;
			if(index == startIndexHistory)
			{
				startIndexHistory = (startIndexHistory+1)%NUM_HIST;
			}
			else
			{
				++historySize;
			}

			endIndexHistory = index;
#ifdef PRINT
			std::cout << "Added new Vehicle with ID " << ID << std::endl;
#endif
			currentHistoryOnDevice = false;
#if SZENARIO == 6
			h_convoyCheck[convoyCheckSize] = h_vehicleSim[i];
#else
			h_convoyCheck[convoyCheckSize] = h_vehicles[i];
#endif
			++convoyCheckSize;
		}
		else
		{
			//vehicle matched, update

			PointCellDevice* tmp = trackedVehicles.at(trackedVehicles.size() -1 );
			PointCellDevice* update = trackedVehicles.at(minIndex);

#ifdef PRINT
			std::cout << "Update ID " << update->getID() << std::endl;
#endif

			trackedVehicles.at(minIndex) = tmp;
			h_intvlIndex[minIndex] = h_intvlIndex[trackedVehicles.size()-1];
			h_intvlIndex[trackedVehicles.size()-1] = minIndex;
		/*	for(int k=0; k<vehicleCount;k++)
			{
				h_distance[k*MAX_SEGMENTS+minIndex] = h_distance[k*MAX_SEGMENTS+(trackedVehicles.size()-1)];
			}*/
#if SZENARIO == 6
			h_updateData[updateCounter*3] = h_vehicleSim[i].getX();
			h_updateData[updateCounter*3+1] = h_vehicleSim[i].getY();
			h_updateData[updateCounter*3+2] = h_vehicleSim[i].getTheta();
#else
			h_updateData[updateCounter*3] = h_vehicles[i].getX();
			h_updateData[updateCounter*3+1] = h_vehicles[i].getY();
			h_updateData[updateCounter*3+2] = h_vehicles[i].getTheta();
#endif
			trackedVehicles.pop_back();
#ifdef PRINT
			std::cout << "Updated vehicle with ID " << update->getID() << std::endl;
#endif
			updateCheck.push_back(update);
			++updateCounter;
		}
	}
	for(int i=0; i<updateCounter; i++)
	{
		for(int j=0; j<intervalSize; j++)
		{
			if(updateCheck[i] == &h_intervalMap[j])
			{
				h_intvlIndex[i] = j;
				break;
			}
		}
	}
	//Update all matched vehicles
	if(updateCounter >0)
	{
		cudaMemcpyAsync(d_history, history, historySize*sizeof(History), cudaMemcpyHostToDevice, stream4);
		cudaMemcpyAsync(d_intervalMap, h_intervalMap, intervalSize*sizeof(PointCellDevice), cudaMemcpyHostToDevice, stream5);
		cudaMemcpyAsync(d_updataData, h_updateData, updateCounter*3*sizeof(double), cudaMemcpyHostToDevice, stream2);
		cudaMemcpyAsync(d_intvlIndex, h_intvlIndex, updateCounter*sizeof(int), cudaMemcpyHostToDevice, stream3);
		cudaDeviceSynchronize();
		updateKernel<<<updateCounter,25>>>(d_intervalMap, d_updataData, d_intvlIndex);
		findHistoryWithIDDevice<<<historySize,updateCounter>>>(d_history,d_intervalMap,d_intvlIndex,d_IDincluded);
		checkHistoryForDuplicateDevice<<<updateCounter, MAX_LENGTH_HIST_CONV>>>(d_history,d_intervalMap,d_intvlIndex,d_IDincluded,d_duplicate);
		addUpdatedPositionToHistoryDevice<<<1,updateCounter>>>(d_history, d_intervalMap,d_intvlIndex, d_IDincluded,d_duplicate);
		cudaMemcpyAsync(h_intervalMap, d_intervalMap, intervalSize*sizeof(PointCellDevice), cudaMemcpyDeviceToHost, stream3);
		cudaMemcpyAsync(history, d_history, historySize*sizeof(History), cudaMemcpyDeviceToHost, stream4);
		cudaDeviceSynchronize();
		for(int i=0; i<updateCounter;i++)
		{
			h_convoyCheck[convoyCheckSize] = h_intervalMap[h_intvlIndex[i]];
			++convoyCheckSize;
		}
	}

	//delete all tracks that could not be matched
	for(uint k = 0; k < trackedVehicles.size(); k++)
	{
		PointCellDevice* tmp = trackedVehicles.at(k);
		for(uint m = 0; m < intervalSize; m++)
		{
			if(tmp == &h_intervalMap[m])
			{
				h_intervalMap[m] = h_intervalMap[--intervalSize];
				break;
			}
		}
	}

	//add all observations that could not be matched
	for(uint k = 0; k < indicesToAdd.size(); k++)
	{
		if(intervalSize < MAX_SEGMENTS)
		{
#if SZENARIO == 6
			h_intervalMap[intervalSize++] = h_vehicleSim[indicesToAdd.at(k)];
#else
			h_intervalMap[intervalSize++] = h_vehicles[indicesToAdd.at(k)];
#endif
		}
	}

	//find Convoy
	if(historySize > 0 && convoyCheckSize >0)
	{
		cudaMemcpy(d_newVeh, h_convoyCheck, convoyCheckSize*sizeof(PointCellDevice), cudaMemcpyHostToDevice);
		dim3 grid(historySize, convoyCheckSize);
		findConvoyDevice<<<grid, MAX_LENGTH_HIST_CONV>>>(d_newVeh,d_history,d_historyMatch);
		cudaMemcpy(h_historyMatch, d_historyMatch, convoyCheckSize*sizeof(int), cudaMemcpyDeviceToHost);
		for(uint i=0; i<convoyCheckSize;i++)
		{
		//	std::cout << "ID " << h_historyMatch[i]  << std::endl;
			if(h_historyMatch[i] != INT_MAX)
			{
				PointCellDevice vehicle = h_convoyCheck[i];
				double x = vehicle.getX();
				int interval = floor(x);
				int id1 = h_historyMatch[i];
				int id2 = vehicle.getID();
#ifdef PRINT
				std::cout << "ID1 " << id1  << " ID2 " << id2 << std::endl;
#endif
				bool convoyFound = false;
				if(convoySize >0)
				{
					cudaMemcpy(d_convoys, convoys, convoySize*sizeof(Convoy), cudaMemcpyHostToDevice);
					cudaDeviceSynchronize();
					findIDInConvoyDevice<<<convoySize, MAX_LENGTH_HIST_CONV,0,stream3>>>(d_convoys, d_IDincluded,id1,id2);
					checkConvoyForDuplicateDevice<<<convoySize, MAX_LENGTH_HIST_CONV,0,stream2>>>(d_convoys, &(d_newVeh[i]),d_duplicate);
					cudaMemcpyAsync(h_IDincluded, d_IDincluded, convoySize*2*sizeof(int), cudaMemcpyDeviceToHost, stream3);
					cudaMemcpyAsync(h_duplicate, d_duplicate, convoySize*sizeof(bool), cudaMemcpyDeviceToHost, stream2);
					cudaDeviceSynchronize();
				}
				for(uint j = startIndexConvoys; j != endIndexConvoys; j = (j+1)%NUM_CONV)
				{
			//		int it1, it2;
					Convoy currentConvoy = convoys[j];
			//		it1 = findIDinConvoy(currentConvoy, id1);
			//		it2 = findIDinConvoy(currentConvoy, id2);
					int it1 = h_IDincluded[j*2];
					int it2 = h_IDincluded[j*2+1];
				/*	if(it1 != it11 || it2 != it21)
					{
						std::cout << "Included CPU 1: " << it1 << " CPU2: " << it2 << " GPU1: " << it11 << " GPU2: " << it21 << std::endl;
					}*/
					if(it1 != INT_MAX && it2 != INT_MAX)
					{
						//convoy already exists with both IDS
						//check if this x value is already contained
						if(h_duplicate[j]/*checkConvoyForDuplicate(interval+0.5, currentConvoy)*/)
						{
							//x value is not contained
							int index = (currentConvoy.endIndexTracks+1)%MAX_LENGTH_HIST_CONV;
							convoys[j].tracks[currentConvoy.endIndexTracks].x = interval+0.5;
							convoys[j].tracks[currentConvoy.endIndexTracks].y = vehicle.getY();
							convoys[j].tracks[currentConvoy.endIndexTracks].theta = vehicle.getTheta();
							convoys[j].tracks[currentConvoy.endIndexTracks].subIntvl = 0.5;
							convoys[j].endIndexTracks = index;
							if(index == convoys[j].startIndexTracks)
							{
								convoys[j].startIndexTracks = (convoys[j].startIndexTracks+1)%MAX_LENGTH_HIST_CONV;
							}
							if(interval+0.5 > convoys[j].highestValue.x)
							{
								convoys[j].highestValue.x = interval+0.5;
								convoys[j].highestValue.y = vehicle.getY();
								convoys[j].highestValue.theta = vehicle.getTheta();
								convoys[j].highestValue.subIntvl = 0.5;
							}
						}
						convoyFound = true;
#ifdef PRINT
						std::cout << "existing Convoy with ID " << convoys[j].ID << " y " << vehicle.getY() << " startIndexTracks: " << convoys[j].startIndexTracks <<" EndindexTracks: "<< convoys[j].endIndexTracks<< std::endl;
#endif
						break;
					}
				}
				if(convoyFound)
				{
					continue;
				}
				for(uint j = startIndexConvoys; j != endIndexConvoys; j = (j+1)%NUM_CONV)
				{
		//			int it1, it2;
					Convoy currentConvoy = convoys[j];
		//			it1 = findIDinConvoy(currentConvoy, id1);
		//			it2 = findIDinConvoy(currentConvoy, id2);
					int it1 = h_IDincluded[j*2];
					int it2 = h_IDincluded[j*2+1];
					if (it1 != INT_MAX)
					{
						int index = (currentConvoy.endIndexTracks+1)%MAX_LENGTH_HIST_CONV;
						//check if this x value is already contained
						if(h_duplicate[j]/*checkConvoyForDuplicate(interval+0.5, currentConvoy)*/)
						{
							convoys[j].tracks[currentConvoy.endIndexTracks].x = interval+0.5;
							convoys[j].tracks[currentConvoy.endIndexTracks].y = vehicle.getY();
							convoys[j].tracks[currentConvoy.endIndexTracks].theta = vehicle.getTheta();
							convoys[j].tracks[currentConvoy.endIndexTracks].subIntvl = 0.5;
							convoys[j].endIndexTracks = index;
							if(index == convoys[j].startIndexTracks)
							{
								convoys[j].startIndexTracks = (convoys[j].startIndexTracks+1)%MAX_LENGTH_HIST_CONV;
							}
							if(interval+0.5 > convoys[j].highestValue.x)
							{
								convoys[j].highestValue.x = interval+0.5;
								convoys[j].highestValue.y = vehicle.getY();
								convoys[j].highestValue.theta = vehicle.getTheta();
								convoys[j].highestValue.subIntvl = 0.5;
							}
						}
						int IDindex = (currentConvoy.endIndexID+1)%MAX_LENGTH_HIST_CONV;
						convoys[j].participatingVehicles[currentConvoy.endIndexID] = id2;
						convoys[j].endIndexID = IDindex;
						if(IDindex == convoys[j].startIndexID)
						{
							convoys[j].startIndexID = (convoys[j].startIndexID+1)%MAX_LENGTH_HIST_CONV;
						}
						convoyFound = true;
#ifdef PRINT
						std::cout << "existing Convoy with ID " << convoys[j].ID << " y " << vehicle.getY() << " startIndexTracks: " << convoys[j].startIndexTracks <<" EndindexTracks: "<< convoys[j].endIndexTracks<< std::endl;
#endif
						break;

					}
					else if (it2 != INT_MAX)
					{
						//only add position to convoy if it will be the highest value or the difference in y is not so big
						if(interval+0.5 < convoys[j].highestValue.x && !checkConvoyForY(vehicle.getY(),interval +0.5,currentConvoy))
						{
							continue;
						}
						int index = (currentConvoy.endIndexTracks+1)%MAX_LENGTH_HIST_CONV;
						//check if this x value is already contained
						if(h_duplicate[j]/*checkConvoyForDuplicate(interval+0.5, currentConvoy)*/)
						{
							convoys[j].tracks[currentConvoy.endIndexTracks].x = interval+0.5;
							convoys[j].tracks[currentConvoy.endIndexTracks].y = vehicle.getY();
							convoys[j].tracks[currentConvoy.endIndexTracks].theta = vehicle.getTheta();
							convoys[j].tracks[currentConvoy.endIndexTracks].subIntvl = 0.5;
							convoys[j].endIndexTracks = index;
							if(index == convoys[j].startIndexTracks)
							{
								convoys[j].startIndexTracks = (convoys[j].startIndexTracks+1)%MAX_LENGTH_HIST_CONV;
							}
							if(interval+0.5 > convoys[j].highestValue.x)
							{
								convoys[j].highestValue.x = interval+0.5;
								convoys[j].highestValue.y = vehicle.getY();
								convoys[j].highestValue.theta = vehicle.getTheta();
								convoys[j].highestValue.subIntvl = 0.5;
							}
						}
						int IDindex = (currentConvoy.endIndexID+1)%MAX_LENGTH_HIST_CONV;
						convoys[j].participatingVehicles[currentConvoy.endIndexID] = id1;
						convoys[j].endIndexID = IDindex;
						if(IDindex == convoys[j].startIndexID)
						{
							convoys[j].startIndexID = (convoys[j].startIndexID+1)%MAX_LENGTH_HIST_CONV;
						}
						convoyFound = true;
#ifdef PRINT
						std::cout << "existing Convoy with ID " << convoys[j].ID << " y " << vehicle.getY() << " startIndexTracks: " << convoys[j].startIndexTracks <<" EndindexTracks: "<< convoys[j].endIndexTracks<< std::endl;
#endif
						break;
					}
				}

				if(!convoyFound)
				{
					int cIndex = endIndexConvoys;
					convoys[cIndex].ID = convoyID++;
					convoys[cIndex].participatingVehicles[0] = id1;
					convoys[cIndex].participatingVehicles[1] = id2;
					convoys[cIndex].startIndexID = 0;
					convoys[cIndex].endIndexID = 2;
					convoys[cIndex].startIndexTracks = 0;
					convoys[cIndex].endIndexTracks = 1;
					convoys[cIndex].tracks[0].x = interval+0.5;
					convoys[cIndex].tracks[0].y = vehicle.getY();
					convoys[cIndex].tracks[0].theta = vehicle.getTheta();
					convoys[cIndex].tracks[0].subIntvl = 0.5;
					endIndexConvoys = (endIndexConvoys+1)%NUM_CONV;
					convoys[cIndex].highestValue.x = interval+0.5;
					convoys[cIndex].highestValue.y = vehicle.getY();
					convoys[cIndex].highestValue.theta = vehicle.getTheta();
					convoys[cIndex].highestValue.subIntvl = 0.5;

					if(convoySize == NUM_CONV)
					{
						startIndexConvoys = (startIndexConvoys+1)%NUM_CONV;
					}
					else
					{
						++convoySize;
					}
#ifdef PRINT
					std::cout << "new Convoy with ID " << convoyID-1 << " containing "<< id1 << " , " << id2 << std::endl;
#endif
				}
				currentConvoyOnDevice = false;
			}
		}
	}
}

void ConvoyTracker::findConvoy(PointCellDevice vehicle) {
	double x = vehicle.getX();
	int interval = floor(x);
	for(int i=startIndexHistory; i != endIndexHistory; i = (i+1)%NUM_HIST)
	{
		if (history[i].ID == vehicle.getID())
		{
			continue;
		}

		for (uint j = history[i].startIndex; i != history[i].endIndex; i = (i+1)%MAX_LENGTH_HIST_CONV)
		{
			EMLPos pc = history[i].tracks[j];
			if (pc.x - 0.5 <= vehicle.getX()
					&& vehicle.getX() <= pc.x + 0.5) {
				if (pc.y - 1.0 <= vehicle.getY() && vehicle.getY() <= pc.y + 1.0) {
					//current vehicle position matches with one from history -> add this pointcell to convoy
					int id1 = history[i].ID;
					int id2 = vehicle.getID();
					bool convoyFound = false;
					for (uint j = startIndexConvoys; j != endIndexConvoys; j = (j + 1) % NUM_CONV)
					{
						int it1, it2;
						Convoy currentConvoy = convoys[j];
						it1 = findIDinConvoy(currentConvoy, id1);
						it2 = findIDinConvoy(currentConvoy, id2);
						if (it1 != INT_MAX && it2 != INT_MAX) {
							//convoy already exists with both IDS
							//check if this x value is already contained
							if (checkConvoyForDuplicate(interval + 0.5,currentConvoy)) {
								//x value is not contained
								int index = (currentConvoy.endIndexTracks + 1) % MAX_LENGTH_HIST_CONV;
								convoys[j].tracks[index].x = interval + 0.5;
								convoys[j].tracks[index].y = vehicle.getY();
								convoys[j].tracks[index].theta = vehicle.getTheta();
								convoys[j].tracks[index].subIntvl = 0.5;
								convoys[j].endIndexTracks = index;
							}
							convoyFound = true;
							break;
						} else if (it1 != INT_MAX) {
							int index = (currentConvoy.endIndexTracks + 1)% MAX_LENGTH_HIST_CONV;
							//check if this x value is already contained
							if (checkConvoyForDuplicate(interval + 0.5,
									currentConvoy)) {
								convoys[j].tracks[index].x = interval + 0.5;
								convoys[j].tracks[index].y = vehicle.getY();
								convoys[j].tracks[index].theta = vehicle.getTheta();
								convoys[j].tracks[index].subIntvl = 0.5;
								convoys[j].endIndexTracks = index;
							}
							int IDindex = (currentConvoy.endIndexID + 1)%MAX_LENGTH_HIST_CONV;
							convoys[j].participatingVehicles[IDindex] = vehicle.getID();
							convoys[j].endIndexID = IDindex;
							convoyFound = true;
							break;
						} else if (it2 != INT_MAX) {
							int index = (currentConvoy.endIndexTracks + 1)% MAX_LENGTH_HIST_CONV;
							//check if this x value is already contained
							if (checkConvoyForDuplicate(interval + 0.5, currentConvoy)) {
								convoys[j].tracks[currentConvoy.endIndexTracks].x = interval + 0.5;
								convoys[j].tracks[currentConvoy.endIndexTracks].y = vehicle.getY();
								convoys[j].tracks[currentConvoy.endIndexTracks].theta =vehicle.getTheta();
								convoys[j].tracks[currentConvoy.endIndexTracks].subIntvl = 0.5;
								convoys[j].endIndexTracks = index;
							}
							int IDindex = (currentConvoy.endIndexID + 1)% MAX_LENGTH_HIST_CONV;
							convoys[j].participatingVehicles[currentConvoy.endIndexID] = id1;
							convoys[j].endIndexID = IDindex;
							convoyFound = true;
							break;
						}
					}

					if (!convoyFound) {
						int cIndex = (endIndexConvoys + 1) % NUM_CONV;
						convoys[endIndexConvoys].ID = convoyID++;
						convoys[endIndexConvoys].participatingVehicles[0] = id1;
						convoys[endIndexConvoys].participatingVehicles[1] = id2;
						convoys[endIndexConvoys].startIndexID = 0;
						convoys[endIndexConvoys].endIndexID = 2;
						convoys[endIndexConvoys].startIndexTracks = 0;
						convoys[endIndexConvoys].endIndexTracks = 1;
						convoys[endIndexConvoys].tracks[0].x = interval + 0.5;
						convoys[endIndexConvoys].tracks[0].y = vehicle.getY();
						convoys[endIndexConvoys].tracks[0].theta = vehicle.getTheta();
						convoys[endIndexConvoys].tracks[0].subIntvl = 0.5;
						endIndexConvoys = cIndex;
						if(convoySize < NUM_CONV)
						{
							++convoySize;
						}
						else
						{
							startIndexConvoys = (startIndexConvoys+1)%NUM_CONV;
#ifdef PRINT
							std::cout << "First entry overwrite" << std::endl;
#endif
						}
					}
					currentConvoyOnDevice = false;
					return;
				}
			}
		}
	}
}

/**
 * shifts values of stored convoy tracks and history infos by given amount because it´s values are relative to own vehicle position
 */
void ConvoyTracker::shiftConvoyHistory(double x)
{
	std::vector<int> toDelete;
	//update history
	for (int i = startIndexHistory; i != endIndexHistory; i = (i+1)%NUM_HIST)
	{
		for (uint j = history[i].startIndex; i != history[i].endIndex; i = (i+1)%MAX_LENGTH_HIST_CONV)
		{
			history[i].tracks[j].subIntvl += x;
			int numIntervals = (int) ((history[i].tracks[j].subIntvl) / INTERVALL_LENGTH);
			history[i].tracks[j].x -= numIntervals;
			history[i].tracks[j].subIntvl -= numIntervals;
		}

		//check whether current History is already behind our car
		int endId = (history[i].endIndex-1)%MAX_LENGTH_HIST_CONV;
		if(endId <0)
		{
			endId = MAX_LENGTH_HIST_CONV-1;
		}
		if(history[i].tracks[endId].x < -5)
		{
			//if yes, mark history to delete
			toDelete.push_back(i);
		}
	}

	int end = (endIndexHistory-1)%NUM_HIST;
	if(end < 0)
	{
		end = NUM_HIST-1;
	}
	for(uint i=0; i<toDelete.size(); i++)
	{
		history[toDelete.at(i)] = history[end];
		endIndexHistory = end;
		--historySize;
	}

	toDelete.clear();

	//update Convoys
	for (uint i = startIndexConvoys; i != endIndexConvoys; i = (i + 1) % NUM_CONV)
	{
		for(uint j = convoys[i].startIndexTracks; j != convoys[i].endIndexTracks; j = (j+1)%MAX_LENGTH_HIST_CONV)
		{
			convoys[i].tracks[j].subIntvl += x;
			int numIntervals = (int) ((convoys[i].tracks[j].subIntvl) / INTERVALL_LENGTH);
			convoys[i].tracks[j].x -= numIntervals;
			convoys[i].tracks[j].subIntvl -= numIntervals;
		}

		if(convoys[i].tracks[(convoys[i].endIndexTracks-1) % MAX_LENGTH_HIST_CONV].x < -5)
		{
			toDelete.push_back(i);
		}
	}

	end = ((endIndexConvoys-1)%NUM_CONV);
	if(end <0)
	{
		end = NUM_CONV-1;
	}
	for(uint i=0; i<toDelete.size(); i++)
	{

		convoys[toDelete.at(i)] = convoys[end];
		endIndexConvoys = end;
		--convoySize;
	}
}

/**
 * rotates and updates lateral position of stored convoy tracks and history infos by given amount because it´s values are relative to own vehicle position
 */
void ConvoyTracker::rotateConvoyHistory(double theta, double y)
{
	double angleInRadians = theta*M_PI/180.0;
	double mat[2][2] = { { cos(angleInRadians), -sin(angleInRadians) },
			{ sin(angleInRadians), cos(angleInRadians) } };
	//update history
	for (int i = startIndexHistory; i != endIndexHistory; i = (i+1)%NUM_HIST)
	{
		for (uint j = history[i].startIndex; i != history[i].endIndex; i = (i+1)%MAX_LENGTH_HIST_CONV)
		{
			history[i].tracks[j].y -= y;
			history[i].tracks[j].theta -= angleInRadians;

			double xAbs = history[i].tracks[j].x;
			double yAbs = history[i].tracks[j].y;

			xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
			yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

			history[i].tracks[j].y -= yAbs;
			history[i].tracks[j].subIntvl -= xAbs;
		}
	}

	//update Convoys
	for (uint i = startIndexConvoys; i != endIndexConvoys; i = (i + 1) % NUM_CONV)
	{
		for(uint j = convoys[i].startIndexTracks; j != convoys[i].endIndexTracks; j = (j+1)%MAX_LENGTH_HIST_CONV)
		{
			convoys[i].tracks[j].y -= y;
			convoys[i].tracks[j].theta -= angleInRadians;

			double xAbs = convoys[i].tracks[j].x;
			double yAbs = convoys[i].tracks[j].y;

			xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
			yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

			convoys[i].tracks[j].y -= yAbs;
			convoys[i].tracks[j].subIntvl -= xAbs;
		}
	}
}

void ConvoyTracker::visualizeConvoys()
{
	visualizer.visualizeConvoys(EML, convoys, startIndexConvoys, endIndexConvoys);
}

void ConvoyTracker::visualizeHistory()
{
	visualizer.visualizeHistory(EML, history, startIndexHistory, endIndexHistory);
}


/**
 * 7|				|
 * 6|				|
 * 5|				|
 * 4|				|
 * 3|	   ||	    |
 * 2|				|
 * 1|				|
 * 0|				|
 *
 *
 * shifts the whole structure by the number of intervals that are covered within @param xMotion and the already
 * stored x-position in interval to the bottom to compensate ego motion
 */
void ConvoyTracker::shiftStructure(double xMotion) {
	*xSubInterval += xMotion;
	int numIntervals = (int) (*xSubInterval / INTERVALL_LENGTH);
	*xSubInterval -= numIntervals;
	for (int i = 0; i < numIntervals; i++)
	{
		std::vector<int> toDelete;
		for(uint j = 0; j < intervalSize; j++)
		{
			double x = h_intervalMap[j].getX();
			int interval = floor(x) + CARINTERVAL;
			if(interval == 0)
			{
				//delete content
				toDelete.push_back(j);
				continue;
			}
			h_intervalMap[j].setX(floor(x) - 0.5);
		}

		for(uint j = 0; j < toDelete.size(); j++)
		{
			h_intervalMap[toDelete.at(j)] = h_intervalMap[intervalSize -1];
		//	intervalMap.at(intervalMap.size() -1) = h_intervalMap[toDelete.at(j)];
		//	intervalMap[j] = tmp;
		//	intervalMap.pop_back();
			--intervalSize;
		}
	}

}
/*
 * rotates the whole map by @param angle and shifts the PCs by @param yMotion
 */
void ConvoyTracker::rotateStructure(double angle, double yMotion) {
	//map for temporary storage of PC that should be moved one interval up
	double angleInRadians = angle * M_PI / 180;

	for(uint i = 0; i < intervalSize; i++)
	{
		PointCellDevice currentVehicle = h_intervalMap[i];
		int interval = floor(currentVehicle.getX());

		//1.Step correct directions of stored PCs
		currentVehicle.setY(currentVehicle.getY() - yMotion);
		currentVehicle.setTheta(currentVehicle.getTheta() - angleInRadians);

		//2. compensate rotation
		double xAbs = ( interval - CARINTERVAL + 0.5) * INTERVALL_LENGTH
				- *xSubInterval;
		double yAbs = currentVehicle.getY();


		double mat[2][2] = { { cos(angleInRadians), -sin(angleInRadians) },
				{ sin(angleInRadians), cos(angleInRadians) } };
		xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
		yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

		currentVehicle.setY(currentVehicle.getY() - yAbs);

		if(xAbs > 0.5*INTERVALL_LENGTH)
		{
			currentVehicle.setX(interval + 1.5);
		//	currentVehicle.subInvtl = xAbs - 0.5;
		}
		else if(xAbs < -0.5*INTERVALL_LENGTH)
		{
			currentVehicle.setX(interval - 0.5);
		//	currentVehicle.subInvtl = xAbs + 0.5;
		}
		else
		{
			currentVehicle.subInvtl -= xAbs;
		}
		h_intervalMap[i] = currentVehicle;
	}
}

bool ConvoyTracker::checkConvoyForDuplicate(double x, Convoy c)
{
	for(int i=c.startIndexTracks; i != c.endIndexTracks; i = (i+1)%MAX_LENGTH_HIST_CONV)
	{
		if(c.tracks[i].x == x)
		{
			return false;
		}
	}
	return true;
}

bool ConvoyTracker::checkHistoryForDuplicate(double x, int historyIndex)
{
	for(int i=history[historyIndex].startIndex; i != history[historyIndex].endIndex; i = (i+1)%MAX_LENGTH_HIST_CONV)
	{
		if(history[historyIndex].tracks[i].x == x)
		{
			return false;
		}
	}
	return true;
}
void ConvoyTracker::transformDataToDevice()
{
	cudaMemcpyAsync(d_history, history, historySize*sizeof(History), cudaMemcpyHostToDevice,stream2);
	cudaMemcpyAsync(d_intervalMap, h_intervalMap, intervalSize*sizeof(PointCellDevice), cudaMemcpyHostToDevice,stream4);
	*h_historyMatchSelf = INT_MAX;
	cudaMemcpyAsync(d_historyMatchSelf, h_historyMatchSelf, sizeof(int), cudaMemcpyHostToDevice,stream5);
	cudaMemcpyAsync(d_convoys, convoys, convoySize*sizeof(Convoy), cudaMemcpyHostToDevice,stream3);
	cudaDeviceSynchronize();
}
void ConvoyTracker::transformDataFromDevice()
{
	cudaDeviceSynchronize();
	std::vector<int> toDelete;
	cudaMemcpyAsync(history, d_history, historySize*sizeof(History), cudaMemcpyDeviceToHost,stream2);
	cudaMemcpyAsync(h_intervalMap, d_intervalMap, intervalSize*sizeof(PointCellDevice), cudaMemcpyDeviceToHost,stream4);
	cudaMemcpyAsync(convoys, d_convoys, convoySize*sizeof(Convoy), cudaMemcpyDeviceToHost,stream3);
	cudaDeviceSynchronize();
	if(historySize > 0)
	{
		*h_historyMatchSelf = INT_MAX;
		findConvoyDeviceSelf<<<historySize, MAX_LENGTH_HIST_CONV>>>(d_history, d_historyMatchSelf);
		cudaMemcpy(h_historyMatchSelf, d_historyMatchSelf, sizeof(int), cudaMemcpyDeviceToHost);
		if(*h_historyMatchSelf != INT_MAX)
		{
			findConvoySelf(*h_historyMatchSelf);
		}
	}
	int end;
	for (int i = startIndexHistory; i != endIndexHistory; i = (i+1)%NUM_HIST)
	{
		int endId = (history[i].endIndex-1)%MAX_LENGTH_HIST_CONV;
		if(endId <0)
		{
			endId = MAX_LENGTH_HIST_CONV-1;
		}
		if(history[i].tracks[endId].x < -5)
		{
			//if yes, mark history to delete
#ifdef PRINT
			std::cout << "Delete history with ID " << history[i].ID << std::endl;
#endif
			toDelete.push_back(i);
		}
	}


	if(toDelete.size() > 0)
	{
		for(int i=toDelete.size()-1; i>=0; i--)
		{
			end = (endIndexHistory-1)%NUM_HIST;
			if(end < 0)
			{
				end = NUM_HIST-1;
			}
			if(toDelete.at(i) != end)
			{
				history[toDelete.at(i)] = history[end];
			}
			endIndexHistory = end;
			--historySize;
		}
	}
	toDelete.clear();

	for (int i = startIndexConvoys; i != endIndexConvoys; i = (i + 1) % NUM_CONV)
	{
		end = (convoys[i].endIndexTracks-1) % MAX_LENGTH_HIST_CONV;
		if(end < 0)
		{
			end = MAX_LENGTH_HIST_CONV-1;
		}
		if(convoys[i].highestValue.x < -5)
		{
#ifdef PRINT
			std::cout << "delete convoy with ID " << convoys[i].ID << std::endl;
#endif
			toDelete.push_back(i);
		}
	}
	if(toDelete.size() > 0)
	{
		for(int i=toDelete.size()-1; i >=0; i--)
		{
			end = (endIndexConvoys-1) % NUM_CONV;
			if(end < 0)
			{
				end = NUM_CONV-1;
			}
			convoys[toDelete.at(i)] = convoys[end];
			endIndexConvoys = end;
			--convoySize;
		}
	}

	toDelete.clear();
	for(uint i=0; i<intervalSize;i++)
	{
		if(h_intervalMap[i].getX() < -100)
		{
			toDelete.push_back(i);
		}
	}
	if(toDelete.size() > 0)
	{
		for(int i=toDelete.size()-1; i>=0;i--)
		{
			h_intervalMap[toDelete.at(i)] = h_intervalMap[--intervalSize];
		}
	}
}

/*
 * Check whether given ID is already part of given Convoy
 */
int ConvoyTracker::findIDinConvoy(Convoy c, int id)
{
	for(int i=c.startIndexID; i != c.endIndexID; i = (i+1)%MAX_LENGTH_HIST_CONV)
	{
		if(c.participatingVehicles[i] == id)
		{
			return i;
		}
	}
	return INT_MAX;
}

/**
 * Get index of stored history with given ID
 */
int ConvoyTracker::findHistoryWithID(int id)
{
	for(int i=startIndexHistory; i != endIndexHistory; i = (i+1)%NUM_HIST)
	{
		if(history[i].ID == id)
		{
			return i;
		}
	}
	//should not happen
	return INT_MAX;
}

void ConvoyTracker::findConvoySelf(int ID)
{
	double x = 0;
	int interval = floor(x);
	int id1 = -1;
	int id2 = ID;
#ifdef PRINT
	std::cout << "ID1 " << id1  << " ID2 " << id2 << std::endl;
#endif
	bool convoyFound = false;
	for(uint j = startIndexConvoys; j != endIndexConvoys; j = (j+1)%NUM_CONV)
	{
		int it1, it2;
		Convoy currentConvoy = convoys[j];
		it1 = findIDinConvoy(currentConvoy, id1);
		it2 = findIDinConvoy(currentConvoy, id2);
		if(it1 != INT_MAX && it2 != INT_MAX)
		{
			//convoy already exists with both IDS
			//check if this x value is already contained
			if(checkConvoyForDuplicate(interval+0.5, currentConvoy))
			{
				//x value is not contained
				int index = (currentConvoy.endIndexTracks+1)%MAX_LENGTH_HIST_CONV;
				convoys[j].tracks[currentConvoy.endIndexTracks].x = 0.5;
				convoys[j].tracks[currentConvoy.endIndexTracks].y = 0;
				convoys[j].tracks[currentConvoy.endIndexTracks].theta = 0;
				convoys[j].tracks[currentConvoy.endIndexTracks].subIntvl = 0.5;
				convoys[j].endIndexTracks = index;
				if(index == convoys[j].startIndexTracks)
				{
					convoys[j].startIndexTracks = (convoys[j].startIndexTracks+1)%MAX_LENGTH_HIST_CONV;
				}
				if(interval+0.5 > convoys[j].highestValue.x)
				{
					convoys[j].highestValue.x = 0.5;
					convoys[j].highestValue.y = 0;
					convoys[j].highestValue.theta = 0;
					convoys[j].highestValue.subIntvl = 0.5;
				}
			}
			convoyFound = true;
#ifdef PRINT
			std::cout << "existing Convoy with ID " << convoys[j].ID << std::endl;
#endif
			break;
		}
		else if (it1 != INT_MAX)
		{
			int index = (currentConvoy.endIndexTracks+1)%MAX_LENGTH_HIST_CONV;
			//check if this x value is already contained
			if(checkConvoyForDuplicate(interval+0.5, currentConvoy))
			{
				convoys[j].tracks[currentConvoy.endIndexTracks].x = 0.5;
				convoys[j].tracks[currentConvoy.endIndexTracks].y = 0;
				convoys[j].tracks[currentConvoy.endIndexTracks].theta = 0;
				convoys[j].tracks[currentConvoy.endIndexTracks].subIntvl = 0.5;
				convoys[j].endIndexTracks = index;
				if(index == convoys[j].startIndexTracks)
				{
					convoys[j].startIndexTracks = (convoys[j].startIndexTracks+1)%MAX_LENGTH_HIST_CONV;
				}
				if(interval+0.5 > convoys[j].highestValue.x)
				{
					convoys[j].highestValue.x = 0.5;
					convoys[j].highestValue.y = 0;
					convoys[j].highestValue.theta = 0;
					convoys[j].highestValue.subIntvl = 0.5;
				}
			}
			int IDindex = (currentConvoy.endIndexID+1)%MAX_LENGTH_HIST_CONV;
			convoys[j].participatingVehicles[currentConvoy.endIndexID] = id2;
			convoys[j].endIndexID = IDindex;
			if(IDindex == convoys[j].startIndexID)
			{
				convoys[j].startIndexID = (convoys[j].startIndexID+1)%MAX_LENGTH_HIST_CONV;
			}
			convoyFound = true;
#ifdef PRINT
			std::cout << "existing Convoy with ID " << convoys[j].ID << std::endl;
#endif
			break;

		}
	}

	if(!convoyFound)
	{
		int cIndex = endIndexConvoys;
		convoys[cIndex].ID = convoyID++;
		convoys[cIndex].participatingVehicles[0] = id1;
		convoys[cIndex].participatingVehicles[1] = id2;
		convoys[cIndex].startIndexID = 0;
		convoys[cIndex].endIndexID = 2;
		convoys[cIndex].startIndexTracks = 0;
		convoys[cIndex].endIndexTracks = 1;
		convoys[cIndex].tracks[0].x = 0.5;
		convoys[cIndex].tracks[0].y = 0;
		convoys[cIndex].tracks[0].theta = 0;
		convoys[cIndex].tracks[0].subIntvl = 0.5;
		endIndexConvoys = (endIndexConvoys+1)%NUM_CONV;
		convoys[cIndex].highestValue.x = 0.5;
		convoys[cIndex].highestValue.y = 0;
		convoys[cIndex].highestValue.theta = 0;
		convoys[cIndex].highestValue.subIntvl = 0.5;

		if(convoySize == NUM_CONV)
		{
			startIndexConvoys = (startIndexConvoys+1)%NUM_CONV;
		}
		else
		{
			++convoySize;
		}
#ifdef PRINT
		std::cout << "new Convoy with ID " << convoyID-1 << " containing "<< id1 << " , " << id2 << std::endl;
#endif
	}
}

bool ConvoyTracker::checkConvoyForY(double y, double x, Convoy c)
{
	double min = INT_MAX;
	double dist;
	int index;
	for(int i=c.startIndexTracks; i != c.endIndexTracks; i = (i+1)%MAX_LENGTH_HIST_CONV)
	{
		dist = fabs(c.tracks[i].x - x);
		if(dist < min)
		{
			min = dist;
			index = i;
		}
	}

	dist = fabs(c.tracks[index].y - y);
	if(dist > CONVOY_THRESHOLD_Y)
	{
		return false;
	}
	return true;
}
