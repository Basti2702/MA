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

	//create cuda streams
	cudaStreamCreate(&stream2);
	cudaStreamCreate(&stream3);
	cudaStreamCreate(&stream4);

	//allocate host memory and create device pointers
	error = cudaHostAlloc((void**) &history, NUM_HIST*sizeof(History), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_history_ptr, history, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_historyMatch, MAX_SEGMENTS*sizeof(int), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_historyMatch_ptr, h_historyMatch, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_historyMatchSelf, sizeof(int), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_historyMatchSelf_ptr, h_historyMatchSelf, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}


	error = cudaHostAlloc((void**) &h_intervalMap, MAX_SEGMENTS*sizeof(PointCellDevice), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_intervalMap_ptr, h_intervalMap, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_convoyCheck, MAX_SEGMENTS*sizeof(PointCellDevice), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_newVeh_ptr, h_convoyCheck, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_IDincluded, NUM_HIST*2*sizeof(int), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_IDincluded_ptr, h_IDincluded, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_duplicate, NUM_HIST*sizeof(bool), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_duplicate_ptr, h_duplicate, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_updateData, MAX_SEGMENTS*3*sizeof(float), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_updataData_ptr, h_updateData, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_intvlIndex, MAX_SEGMENTS*sizeof(int), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_intvlIndex_ptr, h_intvlIndex, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_vehicles, MAX_SEGMENTS*sizeof(PointCellDevice), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_vehicles_ptr, h_vehicles, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_distance, MAX_SEGMENTS*MAX_SEGMENTS*sizeof(float), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_distance_ptr, h_distance, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	size_t sizeConv = NUM_CONV;
	sizeConv *= sizeof(Convoy);
	error = cudaHostAlloc((void **) &convoys, sizeConv, cudaHostAllocMapped);
	if(error != cudaSuccess)
	{
		printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &xSubInterval, sizeof(float), cudaHostAllocMapped);
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

	error = cudaHostGetDevicePointer(&d_convoys_ptr, convoys, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}
}

ConvoyTracker::~ConvoyTracker() {
	// free everything
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
	cudaStreamDestroy(stream2);
	cudaStreamDestroy(stream3);
	cudaStreamDestroy(stream4);
}

/**
 * returns the string representation of the given number @param i
 */
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

/*
 * performs the ego motion compensation of the position with the given index in the given history
 */
__device__ void shiftRotateHistory(History* d_pc, float x, float y, float theta, int index)
{
	//check whether index is out of range at this history
	if(((index < d_pc->endIndex)  && (d_pc->endIndex > d_pc->startIndex)) || ((d_pc->endIndex < d_pc->startIndex) && (index != d_pc->endIndex)))
	{
		//shift compensation
		d_pc->tracks[index].subIntvl += x;
		int numIntervals = (int) ((d_pc->tracks[index].subIntvl) / INTERVALL_LENGTH);
		d_pc->tracks[index].x -= numIntervals;
		d_pc->tracks[index].subIntvl -= numIntervals;

		//rotate compensation
		float angleInRadians = theta*((float)M_PI)/180.0f;
		float mat[2][2] = { { cosf(angleInRadians), -sinf(angleInRadians) },
				{ sinf(angleInRadians), cosf(angleInRadians) } };

		d_pc->tracks[index].y -= y;
		d_pc->tracks[index].theta -= angleInRadians;

		float xAbs = d_pc->tracks[index].x;
		float yAbs = d_pc->tracks[index].y;

		xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
		yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

		d_pc->tracks[index].y -= yAbs;
		d_pc->tracks[index].subIntvl -= xAbs;
	}
}

/*
 * performs the ego motion compensation of the position with the given index in the given convoy
 */
__device__ void shiftRotateConvoy(Convoy* d_eml, float x, float y, float theta, int index)
{
	//check whether index is out of range at this history
	if(((index < d_eml->endIndexTracks)  && (d_eml->endIndexTracks > d_eml->startIndexTracks)) || ((d_eml->endIndexTracks < d_eml->startIndexTracks) && (index != d_eml->endIndexTracks)))
	{
		//shift compensation
		d_eml->tracks[index].subIntvl += x;
		int numIntervals = (int) ((d_eml->tracks[index].subIntvl) / INTERVALL_LENGTH);
		d_eml->tracks[index].x -= numIntervals;
		d_eml->tracks[index].subIntvl -= numIntervals;

		//rotate compensation
		float angleInRadians = theta*((float)M_PI)/180.0f;
		float mat[2][2] = { { cosf(angleInRadians), -sinf(angleInRadians) },
				{ sinf(angleInRadians), cosf(angleInRadians) } };

		d_eml->tracks[index].y -= y;
		d_eml->tracks[index].theta -= angleInRadians;

		float xAbs = d_eml->tracks[index].x;
		float yAbs = d_eml->tracks[index].y;

		xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
		yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

		d_eml->tracks[index].y -= yAbs;
		d_eml->tracks[index].subIntvl -= xAbs;
	}
}

/*
 * performs the ego motion compensation of the given pointcell
 */
__device__ void computeIntervalMap(PointCellDevice* d_interval, float xMotion, float yMotion, float angle, float* xSubInterval)
{
	float angleInRadians = angle * ((float)M_PI) / 180.0f;

	//shift compensation
	*xSubInterval += xMotion;
	int numIntervals = (int) (*xSubInterval / INTERVALL_LENGTH);
	*xSubInterval -= numIntervals;
	for (int i = 0; i < numIntervals; i++)
	{
		double x = d_interval->getX();
		int interval = floor(x) + CARINTERVAL;
		if(interval == 0)
		{
			//mark content to delete
			d_interval->setX(-10000);
			continue;
		}
		d_interval->setX(floor(x) - 0.5f);
	}
	int	interval = floor(d_interval->getX());
	//rotation
	//1.Step correct directions of stored PCs
	d_interval->setY(d_interval->getY() - yMotion);
	d_interval->setTheta(d_interval->getTheta() - angleInRadians);

	//2. compensate rotation
	float xAbs = ( interval - CARINTERVAL + 0.5f) * INTERVALL_LENGTH
			- *xSubInterval;
	float yAbs = d_interval->getY();


	float mat[2][2] = { { cosf(angleInRadians), -sinf(angleInRadians) },
			{ sinf(angleInRadians), cosf(angleInRadians) } };
	xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
	yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

	d_interval->setY(d_interval->getY() - yAbs);

	if(xAbs > 0.5*INTERVALL_LENGTH)
	{
		d_interval->setX(interval + 1.5);
	}
	else if(xAbs < -0.5*INTERVALL_LENGTH)
	{
		d_interval->setX(interval - 0.5);
	}
	else
	{
		d_interval->subInvtl -= xAbs;
	}

}

/*
 * checks whether the given vehicle matches with the position at index in the given history
 */
__device__ bool findHistoryMatch(PointCellDevice* trackedVehicles, History* d_history, int index)
{
	bool result = (d_history->ID != trackedVehicles->getID());
	result = (result && (d_history->tracks[index].x - 0.5 <= trackedVehicles->getX()));
	result = (result && (trackedVehicles->getX() <= d_history->tracks[index].x + 0.5));
	result = (result && (d_history->tracks[index].y - 1.0 <= trackedVehicles->getY()));
	result = (result && (trackedVehicles->getY() <= d_history->tracks[index].y + 1.0));
	return result;
}

/*
 * checks whether the own vehicles position matches with the position at index in the given history
 */
__device__ bool findHistoryMatchSelf(History* d_history, int index)
{
	bool result = true;
	result = (result && (d_history->tracks[index].x - 0.5 <= 0));
	result = (result && (0 <= d_history->tracks[index].x + 0.5));
	result = (result && (d_history->tracks[index].y - 1.0 <= 0));
	result = (result && (0 <= d_history->tracks[index].y + 1.0));
	return result;
}

/*
 * performs kalman filter predict step for one matrix element defined by index of the given vehicle
 */
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
	float tmp = 0;
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

/*
 * distribution of the data for each thread
 */
__global__ void compensateEgoMotionMap(PointCellDevice* d_interval, float* d_subIntvl, float x, float y, float angle)
{
	computeIntervalMap(&(d_interval[threadIdx.x]), x, y, angle, d_subIntvl);
}
__global__ void compensateEgoMotionHistory(History* d_history, float x, float y, float angle)
{
	shiftRotateHistory(&(d_history[blockIdx.x]), x, y, angle, threadIdx.x);
}

__global__ void compensateEgoMotionConvoy(Convoy* d_convoy, float x, float y, float angle)
{
	shiftRotateConvoy(&(d_convoy[blockIdx.x]), x, y, angle, threadIdx.x);
}

/*
 * tries to find a match in history for each tracked vehicles
 */
__global__ void findConvoyDevice(PointCellDevice* trackedVehicles, History* d_history, int* d_historyMatch)
{
	//check whether thread is in bounds
	if(((threadIdx.x < d_history[blockIdx.x].endIndex)  && (d_history[blockIdx.x].endIndex > d_history[blockIdx.x].startIndex)) || ((d_history[blockIdx.x].endIndex < d_history[blockIdx.x].startIndex) && (threadIdx.x != d_history[blockIdx.x].endIndex)))
	{
		//check whether given car matches given history position
		if(findHistoryMatch(&(trackedVehicles[blockIdx.y]),&(d_history[blockIdx.x]),threadIdx.x))
		{
			//write the smallest matched history id to memory
			atomicMin(&(d_historyMatch[blockIdx.y]), d_history[blockIdx.x].ID);
		}
	}
}
/*
 * tries to find a match in history for current vehicle position
 */
__global__ void findConvoyDeviceSelf(History* d_history, int* d_historyMatchSelf)
{
	//check whether thread is in bounds
	if(((threadIdx.x < d_history[blockIdx.x].endIndex)  && (d_history[blockIdx.x].endIndex > d_history[blockIdx.x].startIndex)) || ((d_history[blockIdx.x].endIndex < d_history[blockIdx.x].startIndex) && (threadIdx.x != d_history[blockIdx.x].endIndex)))
	{
		//check wether given history position matches (0,0)
		if(findHistoryMatchSelf(&(d_history[blockIdx.x]),threadIdx.x))
		{
			//write the smallest matched history id to memory
			atomicMin(d_historyMatchSelf, d_history[blockIdx.x].ID);
		}
	}
}

/*
 * set memory for the findConvoy kernel
 */
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
__device__ void updateDevice(PointCellDevice* d_interval, int index, float velocity, float phi, float xNew, float yNew, float thetaNew)
{
	//row
	int i = index / 5;
	//column
	int j = index % 5;

	float tmp = 0;

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
__global__ void updateKernel(PointCellDevice* d_intvl, float* d_updateData, int* d_intvlIndex)
{

	int index = d_intvlIndex[blockIdx.x];
	float xNew = d_updateData[blockIdx.x*3];
	float yNew = d_updateData[blockIdx.x*3+1];
	float thetaNew = d_updateData[blockIdx.x*3+2];
/*	if(threadIdx.x == 0)
	{
		printf("Update ID %d with x %f y %f theta %f, updateValues x %f y %f theta %f\n", d_intvl[index].getID(),d_intvl[index].getX(),d_intvl[index].getY(),d_intvl[index].getTheta(),xNew, yNew,thetaNew);

	}*/
	float x = d_intvl[index].data[5];
	float y = d_intvl[index].data[6];
	float theta = d_intvl[index].data[7];
	float velocity = sqrtf((xNew - x) * (xNew - x) + (yNew - y)*(yNew - y)) / TIMESTAMP;
	float phi = (thetaNew-theta) / TIMESTAMP;
	if(threadIdx.x == 0)
	{
		d_intvl[index].setVelocity(velocity);
		d_intvl[index].setPhi(phi);
	}
	updateDevice(&(d_intvl[index]),threadIdx.x,velocity, phi,xNew,yNew,thetaNew);
}

/*
 * marks for every convoy if the given ids are included in it or not
 */
__global__ void findIDInConvoyDevice(Convoy* d_convoy, int* d_IDIncluded, int id1, int id2)
{
	//check whether thread is in bounds
	if(((threadIdx.x < d_convoy[blockIdx.x].endIndexID)  && (d_convoy[blockIdx.x].endIndexID > d_convoy[blockIdx.x].startIndexID)) || ((d_convoy[blockIdx.x].endIndexID < d_convoy[blockIdx.x].startIndexID) && (threadIdx.x != d_convoy[blockIdx.x].endIndexID)))
	{
		int index = blockIdx.x*2;
		//init memory
		d_IDIncluded[index] = INT_MAX;
		d_IDIncluded[index+1] = INT_MAX;
		__syncthreads();
		//check and write results
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
		//if current convoy is the ego convoy, mark it with INT_MIN
		result = (d_convoy[blockIdx.x].participatingVehicles[threadIdx.x] == -1);
		if(result)
		{
			atomicMin(&(d_IDIncluded[index+1]), INT_MIN);
			atomicMin(&(d_IDIncluded[index]), INT_MIN);
		}
	}
}
/*
 * checks for every convoy whether the given vehicle position is already included or not
 */
__global__ void checkConvoyForDuplicateDevice(Convoy* d_convoy, PointCellDevice* d_vehicle, bool* d_duplicate)
{
	//check whether thread is in bounds
	if(((threadIdx.x < d_convoy[blockIdx.x].endIndexTracks)  && (d_convoy[blockIdx.x].endIndexTracks > d_convoy[blockIdx.x].startIndexTracks)) || ((d_convoy[blockIdx.x].endIndexTracks < d_convoy[blockIdx.x].startIndexTracks) && (threadIdx.x != d_convoy[blockIdx.x].endIndexTracks)))
	{
		d_duplicate[blockIdx.x] = true;
		bool result = (d_convoy[blockIdx.x].tracks[threadIdx.x].x != (floor(d_vehicle->getX())+0.5));
		if(!result)
		{
			d_duplicate[blockIdx.x] = d_duplicate[blockIdx.x] && result;
		}
	}
}
/*
 * checks for every convoy whether own vehicle position is already included or not
 */
__global__ void checkConvoyForDuplicateDeviceSelf(Convoy* d_convoy, bool* d_duplicate)
{
	//check whether thread is in bounds
	if(((threadIdx.x < d_convoy[blockIdx.x].endIndexTracks)  && (d_convoy[blockIdx.x].endIndexTracks > d_convoy[blockIdx.x].startIndexTracks)) || ((d_convoy[blockIdx.x].endIndexTracks < d_convoy[blockIdx.x].startIndexTracks) && (threadIdx.x != d_convoy[blockIdx.x].endIndexTracks)))
	{
		d_duplicate[blockIdx.x] = true;
		bool result = (d_convoy[blockIdx.x].tracks[threadIdx.x].x != 0.5);
		if(!result)
		{
			d_duplicate[blockIdx.x] = d_duplicate[blockIdx.x] && result;
		}
	}
}
/*
 * checks for every history whether own vehicle position is already included or not
 */
__global__ void checkHistoryForDuplicateDevice(History* d_history, PointCellDevice* d_intvl, int* d_intvlIndex, int* d_IDincluded, bool* d_duplicate)
{
	//check whether thread is in bounds
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
/*
 * tries to find the index of the history corresponding to each car
 */
__global__ void findHistoryWithIDDevice(History* d_history, PointCellDevice* d_intvl, int* d_intvlIndex, int* d_IDincluded)
{
	int index = d_intvlIndex[threadIdx.x];
	int ID = d_intvl[index].getID();
	if(d_history[blockIdx.x].ID == ID)
	{
		//write index to memory
		d_IDincluded[threadIdx.x] = blockIdx.x;
	}
}

/*
 * adds updatet positions to corresponding histories
 */
__global__ void addUpdatedPositionToHistoryDevice(History* d_history, PointCellDevice* d_intvl, int* d_intvlIndex, int* d_IDincluded, bool* d_duplicate)
{
	int intvl = floor(d_intvl[d_intvlIndex[threadIdx.x]].getX());
	d_intvl[d_intvlIndex[threadIdx.x]].setX(intvl+ 0.5);
	int historyIndex = d_IDincluded[threadIdx.x];

	//look up whether current position is a duplicate
	if(d_duplicate[threadIdx.x])
	{
		int index = d_history[historyIndex].endIndex;
		d_history[historyIndex].tracks[index].subIntvl = 0.5;
		d_history[historyIndex].tracks[index].x = d_intvl[d_intvlIndex[threadIdx.x]].getX();
		d_history[historyIndex].tracks[index].y = d_intvl[d_intvlIndex[threadIdx.x]].getY();
		d_history[historyIndex].tracks[index].theta = d_intvl[d_intvlIndex[threadIdx.x]].getTheta();
		index = (index+1)%MAX_LENGTH_HIST_CONV;
		d_history[historyIndex].endIndex = index;
		//if number of position exceeds limit, delete oldest position
		if(index == d_history[historyIndex].startIndex)
		{
			d_history[historyIndex].startIndex = (d_history[historyIndex].startIndex+1)%NUM_HIST;
		}
	}
}

int main()
{
	//comment in define in data.cuh to enable laser simulation
#ifdef CREATE_MEASURES
	PGMReader pgmReader;
	double speed = 4.0/3.0;
	for(int i=0; i<NUM_MEASUREMENT; i++)
	{
		std::string number = getNextMeasureAsString(i);
		pgmReader.simulateLaserRays(number);

		//comment in for automatic generation of eml files for a straight drive with 120km/h
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
	error = cudaHostAlloc((void**) &tracker.h_vehicleSim, MAX_SEGMENTS*sizeof(PointCellDevice), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&tracker.d_vehicleSim_ptr, tracker.h_vehicleSim, 0);
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
	}
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
		float angleInRadians = deltaYaw * ((float)M_PI) / 180.0f;
		float mat[2][2] = { { cosf(angleInRadians), -sinf(angleInRadians) },
				{ sinf(angleInRadians), cosf(angleInRadians) } };

		//call ego compensation kernels regarding to current sizes
			if(tracker.historySize > 0)
			{
				compensateEgoMotionHistory<<<tracker.historySize, MAX_LENGTH_HIST_CONV,0, tracker.stream4>>>(tracker.d_history_ptr, deltaX, deltaY, deltaYaw);
			}
			//read new laserdate and extract vehicles
			vehicleCount = tracker.reader.processLaserData(number,tracker.getCurrentSpeed(), tracker.getCurrentYawRate(), tracker.h_vehicles);
			if(tracker.convoySize > 0)
			{
				compensateEgoMotionConvoy<<<tracker.convoySize, MAX_LENGTH_HIST_CONV,0, tracker.stream2>>>(tracker.d_convoys_ptr, deltaX, deltaY, deltaYaw);
				//compensate ego motion to highest value as well
				for(uint k = 0; k < tracker.convoySize; k++)
				{
					tracker.convoys[k].highestValue.subIntvl += deltaX;
					int numIntervals = (int) ((tracker.convoys[k].highestValue.subIntvl) / INTERVALL_LENGTH);
					tracker.convoys[k].highestValue.x -= numIntervals;
					tracker.convoys[k].highestValue.subIntvl -= numIntervals;

					tracker.convoys[k].highestValue.y -= deltaY;
					tracker.convoys[k].highestValue.theta -= angleInRadians;

					float xAbs = tracker.convoys[k].highestValue.x;
					float yAbs = tracker.convoys[k].highestValue.y;

					xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
					yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

					tracker.convoys[k].highestValue.y -= yAbs;
					tracker.convoys[k].highestValue.subIntvl -= xAbs;
				}
			}
			//compesate ego motion interval map and perform pedrict step
			if(tracker.intervalSize > 0)
			{
				compensateEgoMotionMap<<<1,tracker.intervalSize,0,tracker.stream3>>>(tracker.d_intervalMap_ptr, tracker.d_subIntvl_ptr, deltaX, deltaY, deltaYaw);
				predict<<<tracker.intervalSize,25,0,tracker.stream3>>>(tracker.d_intervalMap_ptr);

			}

			//search histories for match with ego position
			if(tracker.historySize > 0)
			{
				*tracker.h_historyMatchSelf = INT_MAX;
				findConvoyDeviceSelf<<<tracker.historySize, MAX_LENGTH_HIST_CONV>>>(tracker.d_history_ptr, tracker.d_historyMatchSelf_ptr);

				cudaDeviceSynchronize();
				if(*tracker.h_historyMatchSelf != INT_MAX)
				{
					tracker.findConvoySelf(*tracker.h_historyMatchSelf);
				}
			}

		tracker.transformDataFromDevice();
		cudaStreamSynchronize(tracker.stream3);
		//write adress of each pointcell to vector
		for(uint j=0; j<tracker.intervalSize;j++)
		{
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
 * @param trackedVehicles: contains pointers to all vehicles stored in the interval map
 * @param vehicels: contains all observed vehicles from the current laser file
 */
void ConvoyTracker::associateAndUpdate(int vehicleCount, std::vector<PointCellDevice*> trackedVehicles)
{
	//initialize all IDs in possible history to -1 to have no false detection in findConvoy
	memSetHistoryMatch<<<1,MAX_SEGMENTS,0,stream2>>>(d_historyMatch_ptr);

	convoyCheckSize = 0;
	int updateCounter = 0;
	int indexCounter = trackedVehicles.size();
	std::vector<int> indicesToAdd;
	std::vector<PointCellDevice*> updateCheck;

	for(uint i = 0; i<vehicleCount; i++)
	{
		//get values from observation
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
			//compute distance to stored vehicle
			double x1 = trackedVehicles.at(j)->getX();
			double y1 = trackedVehicles.at(j)->getY();
			double theta1 = trackedVehicles.at(j)->getTheta();
#ifdef PRINT
			std::cout << "X1: " << x1 << " Y1: " << y1<< " Theta1: " << theta1 <<std::endl;
#endif
			double dist = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1) + (theta - theta1)*(theta - theta1));

			//find vehicle with smallest distance
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
			history[endIndexHistory].tracks[0].subIntvl = 0.5f;
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
			//vehicle matched, mark for update

			PointCellDevice* tmp = trackedVehicles.at(trackedVehicles.size() -1 );
			PointCellDevice* update = trackedVehicles.at(minIndex);

#ifdef PRINT
			std::cout << "Update ID " << update->getID() << std::endl;
#endif

			trackedVehicles.at(minIndex) = tmp;
			h_intvlIndex[minIndex] = h_intvlIndex[trackedVehicles.size()-1];
			h_intvlIndex[trackedVehicles.size()-1] = minIndex;

			//save update data for later use
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
	//get interval index for all updatet pointers
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
		updateKernel<<<updateCounter,25>>>(d_intervalMap_ptr, d_updataData_ptr, d_intvlIndex_ptr);
		findHistoryWithIDDevice<<<historySize,updateCounter>>>(d_history_ptr,d_intervalMap_ptr,d_intvlIndex_ptr,d_IDincluded_ptr);
		checkHistoryForDuplicateDevice<<<updateCounter, MAX_LENGTH_HIST_CONV>>>(d_history_ptr,d_intervalMap_ptr,d_intvlIndex_ptr,d_IDincluded_ptr,d_duplicate_ptr);
		addUpdatedPositionToHistoryDevice<<<1,updateCounter>>>(d_history_ptr, d_intervalMap_ptr,d_intvlIndex_ptr, d_IDincluded_ptr,d_duplicate_ptr);
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
		dim3 grid(historySize, convoyCheckSize);
		findConvoyDevice<<<grid, MAX_LENGTH_HIST_CONV>>>(d_newVeh_ptr,d_history_ptr,d_historyMatch_ptr);
		cudaDeviceSynchronize();
		for(uint i=0; i<convoyCheckSize;i++)
		{
			//look up if vehicle i matched history position
			if(h_historyMatch[i] != INT_MAX)
			{
				//get vehicle i
				PointCellDevice vehicle = h_convoyCheck[i];
				float x = vehicle.getX();
				int interval = floor(x);
				//get history id
				int id1 = h_historyMatch[i];
				int id2 = vehicle.getID();
#ifdef PRINT
				std::cout << "ID1 " << id1  << " ID2 " << id2 << std::endl;
#endif
				bool convoyFound = false;
				if(convoySize >0)
				{	//find convoy and duplicates on device
					findIDInConvoyDevice<<<convoySize, MAX_LENGTH_HIST_CONV,0,stream3>>>(d_convoys_ptr, d_IDincluded_ptr,id1,id2);
					checkConvoyForDuplicateDevice<<<convoySize, MAX_LENGTH_HIST_CONV,0,stream2>>>(d_convoys_ptr, &(d_newVeh_ptr[i]),d_duplicate_ptr);
					cudaDeviceSynchronize();
				}
				for(uint j = startIndexConvoys; j != endIndexConvoys; j = (j+1)%NUM_CONV)
				{
					Convoy currentConvoy = convoys[j];
					int it1 = h_IDincluded[j*2];
					int it2 = h_IDincluded[j*2+1];
					if(it1 == INT_MIN || it2 == INT_MIN)
					{
						continue;
					}
					if(it1 != INT_MAX && it2 != INT_MAX)
					{
						//convoy already exists with both IDS
						//check if this x value is already contained
						if(h_duplicate[j])
						{
							//x value is not contained
							int index = (currentConvoy.endIndexTracks+1)%MAX_LENGTH_HIST_CONV;
							convoys[j].tracks[currentConvoy.endIndexTracks].x = interval+0.5f;
							convoys[j].tracks[currentConvoy.endIndexTracks].y = vehicle.getY();
							convoys[j].tracks[currentConvoy.endIndexTracks].theta = vehicle.getTheta();
							convoys[j].tracks[currentConvoy.endIndexTracks].subIntvl = 0.5f;
							convoys[j].endIndexTracks = index;
							if(index == convoys[j].startIndexTracks)
							{
								convoys[j].startIndexTracks = (convoys[j].startIndexTracks+1)%MAX_LENGTH_HIST_CONV;
							}
							if(interval+0.5 > convoys[j].highestValue.x)
							{
								convoys[j].highestValue.x = interval+0.5f;
								convoys[j].highestValue.y = vehicle.getY();
								convoys[j].highestValue.theta = vehicle.getTheta();
								convoys[j].highestValue.subIntvl = 0.5f;
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
					Convoy currentConvoy = convoys[j];
					int it1 = h_IDincluded[j*2];
					int it2 = h_IDincluded[j*2+1];
					if(it1 == INT_MIN || it2 == INT_MIN)
					{
						continue;
					}
					if (it1 != INT_MAX)
					{
						int index = (currentConvoy.endIndexTracks+1)%MAX_LENGTH_HIST_CONV;
						//check if this x value is already contained
						if(h_duplicate[j])
						{
							convoys[j].tracks[currentConvoy.endIndexTracks].x = interval+0.5f;
							convoys[j].tracks[currentConvoy.endIndexTracks].y = vehicle.getY();
							convoys[j].tracks[currentConvoy.endIndexTracks].theta = vehicle.getTheta();
							convoys[j].tracks[currentConvoy.endIndexTracks].subIntvl = 0.5f;
							convoys[j].endIndexTracks = index;
							if(index == convoys[j].startIndexTracks)
							{
								convoys[j].startIndexTracks = (convoys[j].startIndexTracks+1)%MAX_LENGTH_HIST_CONV;
							}
							if(interval+0.5 > convoys[j].highestValue.x)
							{
								convoys[j].highestValue.x = interval+0.5f;
								convoys[j].highestValue.y = vehicle.getY();
								convoys[j].highestValue.theta = vehicle.getTheta();
								convoys[j].highestValue.subIntvl = 0.5f;
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
						if(interval+0.5 < convoys[j].highestValue.x && !checkConvoyForY(vehicle.getY(),interval+0.5f,currentConvoy))
						{
							continue;
						}
						int index = (currentConvoy.endIndexTracks+1)%MAX_LENGTH_HIST_CONV;
						//check if this x value is already contained
						if(h_duplicate[j])
						{
							convoys[j].tracks[currentConvoy.endIndexTracks].x = interval+0.5f;
							convoys[j].tracks[currentConvoy.endIndexTracks].y = vehicle.getY();
							convoys[j].tracks[currentConvoy.endIndexTracks].theta = vehicle.getTheta();
							convoys[j].tracks[currentConvoy.endIndexTracks].subIntvl = 0.5f;
							convoys[j].endIndexTracks = index;
							if(index == convoys[j].startIndexTracks)
							{
								convoys[j].startIndexTracks = (convoys[j].startIndexTracks+1)%MAX_LENGTH_HIST_CONV;
							}
							if(interval+0.5 > convoys[j].highestValue.x)
							{
								convoys[j].highestValue.x = interval+0.5f;
								convoys[j].highestValue.y = vehicle.getY();
								convoys[j].highestValue.theta = vehicle.getTheta();
								convoys[j].highestValue.subIntvl = 0.5f;
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
				//if now convoy matche our needs, create new one
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
					convoys[cIndex].tracks[0].x = interval+0.5f;
					convoys[cIndex].tracks[0].y = vehicle.getY();
					convoys[cIndex].tracks[0].theta = vehicle.getTheta();
					convoys[cIndex].tracks[0].subIntvl = 0.5f;
					endIndexConvoys = (endIndexConvoys+1)%NUM_CONV;
					convoys[cIndex].highestValue.x = interval+0.5f;
					convoys[cIndex].highestValue.y = vehicle.getY();
					convoys[cIndex].highestValue.theta = vehicle.getTheta();
					convoys[cIndex].highestValue.subIntvl = 0.5f;

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

/*
 * calls the visualizer to draw the convoys
 */
void ConvoyTracker::visualizeConvoys()
{
	visualizer.visualizeConvoys(EML, convoys, startIndexConvoys, endIndexConvoys);
}
/*
 * calls the visualizer to draw the historys
 */
void ConvoyTracker::visualizeHistory()
{
	visualizer.visualizeHistory(EML, history, startIndexHistory, endIndexHistory);
}
/*
 * checks the transformed data from device, deletes entries if necessary
 */
void ConvoyTracker::transformDataFromDevice()
{
	std::vector<int> toDelete;

	int end;
	//check history for delete
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

//delete history
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

	//check convoys for delete
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
	//delete convoys
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

	//check interval map for delete
	for(uint i=0; i<intervalSize;i++)
	{
		if(h_intervalMap[i].getX() < -100)
		{
			toDelete.push_back(i);
		}
	}

	//deltete pointcells
	if(toDelete.size() > 0)
	{
		for(int i=toDelete.size()-1; i>=0;i--)
		{
			h_intervalMap[toDelete.at(i)] = h_intervalMap[--intervalSize];
		}
	}
}

/*
 * Adds own position to convoy
 */
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
	if(convoySize >0)
	{
		//findConvoy and duplicateCheck for all convoys
		findIDInConvoyDevice<<<convoySize, MAX_LENGTH_HIST_CONV,0,stream3>>>(d_convoys_ptr, d_IDincluded_ptr,id1,id2);
		checkConvoyForDuplicateDeviceSelf<<<convoySize, MAX_LENGTH_HIST_CONV,0,stream2>>>(d_convoys_ptr,d_duplicate_ptr);
		cudaDeviceSynchronize();
	}
	for(uint j = startIndexConvoys; j != endIndexConvoys; j = (j+1)%NUM_CONV)
	{
		Convoy currentConvoy = convoys[j];
		int it1 = h_IDincluded[j*2];
		int it2 = h_IDincluded[j*2+1];
		if(it1 != INT_MAX && it2 != INT_MAX)
		{
			//convoy already exists with both IDS
			//check if this x value is already contained
			if(h_duplicate[j])
			{
				//x value is not contained
				int index = (currentConvoy.endIndexTracks+1)%MAX_LENGTH_HIST_CONV;
				convoys[j].tracks[currentConvoy.endIndexTracks].x = 0.5f;
				convoys[j].tracks[currentConvoy.endIndexTracks].y = 0;
				convoys[j].tracks[currentConvoy.endIndexTracks].theta = 0;
				convoys[j].tracks[currentConvoy.endIndexTracks].subIntvl = 0.5f;
				convoys[j].endIndexTracks = index;
				if(index == convoys[j].startIndexTracks)
				{
					convoys[j].startIndexTracks = (convoys[j].startIndexTracks+1)%MAX_LENGTH_HIST_CONV;
				}
				if(interval+0.5 > convoys[j].highestValue.x)
				{
					convoys[j].highestValue.x = 0.5f;
					convoys[j].highestValue.y = 0;
					convoys[j].highestValue.theta = 0;
					convoys[j].highestValue.subIntvl = 0.5f;
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
			//only self convoy exists
			int index = (currentConvoy.endIndexTracks+1)%MAX_LENGTH_HIST_CONV;
			//check if this x value is already contained
			if(h_duplicate[j])
			{
				convoys[j].tracks[currentConvoy.endIndexTracks].x = 0.5f;
				convoys[j].tracks[currentConvoy.endIndexTracks].y = 0;
				convoys[j].tracks[currentConvoy.endIndexTracks].theta = 0;
				convoys[j].tracks[currentConvoy.endIndexTracks].subIntvl = 0.5f;
				convoys[j].endIndexTracks = index;
				if(index == convoys[j].startIndexTracks)
				{
					convoys[j].startIndexTracks = (convoys[j].startIndexTracks+1)%MAX_LENGTH_HIST_CONV;
				}
				if(interval+0.5 > convoys[j].highestValue.x)
				{
					convoys[j].highestValue.x = 0.5f;
					convoys[j].highestValue.y = 0;
					convoys[j].highestValue.theta = 0;
					convoys[j].highestValue.subIntvl = 0.5f;
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

	//if there was no match, create new one
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
		convoys[cIndex].tracks[0].subIntvl = 0.5f;
		endIndexConvoys = (endIndexConvoys+1)%NUM_CONV;
		convoys[cIndex].highestValue.x = 0.5f;
		convoys[cIndex].highestValue.y = 0;
		convoys[cIndex].highestValue.theta = 0;
		convoys[cIndex].highestValue.subIntvl = 0.5f;

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
/*
 * checks whether the given y value is near enough to next position in x of convoy c
 * should prevent wrong additions in case of a lane change
 */
bool ConvoyTracker::checkConvoyForY(float y, float x, Convoy c)
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
