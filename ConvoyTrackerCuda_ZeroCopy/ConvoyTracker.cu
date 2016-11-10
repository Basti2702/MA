/*
 * ConvoyTracker.cu
 *
 *  Created on: 06.06.2016
 *      Author: Sebastian Reinhart
 */

#include "ConvoyTracker.cuh"

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

	size_t sizeConv = NUM_CONV;
	sizeConv *= sizeof(Convoy);
	error = cudaHostAlloc((void **) &convoys, sizeConv, cudaHostAllocMapped);
	if(error != cudaSuccess)
	{
		printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
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

	error = cudaHostGetDevicePointer(&d_convoys_ptr, convoys, 0);
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

__device__ void shiftRotateConvoy(Convoy* d_eml, double x, double y, double theta, int index)
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

	d_interval->predict();
}

__device__ bool findHistoryMatch(PointCellDevice* trackedVehicles, History* d_history, int index)
{
	bool result = (d_history->ID != trackedVehicles->getID());
	result = (result && (d_history->tracks[index].x - 0.5 <= trackedVehicles->getX()));
	result = (result && (trackedVehicles->getX() <= d_history->tracks[index].x + 0.5));
	result = (result && (d_history->tracks[index].y - 1.0 <= trackedVehicles->getY()));
	result = (result && (trackedVehicles->getY() <= d_history->tracks[index].y + 1.0));
	result = (result && (((index < d_history->endIndex)  && (d_history->endIndex > d_history->startIndex)) || ((d_history->endIndex < d_history->startIndex) && (index != d_history->endIndex))));
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
	result = (result && (((index < d_history->endIndex)  && (d_history->endIndex > d_history->startIndex)) || ((d_history->endIndex < d_history->startIndex) && (index != d_history->endIndex))));
	//result = (result && (index >= d_history->startIndex));
	return result;
}

__global__ void compensateEgoMotionMap(PointCellDevice* d_interval, double* d_subIntvl, double x, double y, double angle)
{
	int index = blockIdx.x*blockDim.x + threadIdx.x;
	computeIntervalMap(&(d_interval[index]), x, y, angle, d_subIntvl);
}
__global__ void compensateEgoMotionHistory(History* d_history, double x, double y, double angle)
{
	shiftRotateHistory(&(d_history[blockIdx.x]), x, y, angle, threadIdx.x);
}

__global__ void compensateEgoMotionConvoy(Convoy* d_convoy, double x, double y, double angle)
{
	shiftRotateConvoy(&(d_convoy[blockIdx.x]), x, y, angle, threadIdx.x);
}

__global__ void compensateEgoMotion(History* d_history, Convoy* d_convoy, PointCellDevice* d_interval, double* d_subIntvl, double x, double y, double angle, int numConv, int intvlSize)
{
	shiftRotateHistory(&(d_history[blockIdx.x]), x, y, angle, threadIdx.x);
	if(blockIdx.x < numConv)
	{
		shiftRotateConvoy(&(d_convoy[blockIdx.x]), x, y, angle, threadIdx.x);
	}
	if(blockIdx.x < intvlSize)
	{
		if(threadIdx.x == 0)
		{
			computeIntervalMap(&(d_interval[blockIdx.x]), x, y, angle, d_subIntvl);

		}
	}
}

__global__ void findConvoyDevice(PointCellDevice* trackedVehicles, History* d_history, int* d_historyMatch)
{
	if(findHistoryMatch(&(trackedVehicles[blockIdx.y]),&(d_history[blockIdx.x]),threadIdx.x))
	{
#ifdef PRINT
		printf("TrackedID %d, HistoryID %d, Index %d\n", trackedVehicles[blockIdx.y].getID(), d_history[blockIdx.x].ID);
#endif
		atomicMin(&(d_historyMatch[blockIdx.y]), d_history[blockIdx.x].ID);
	}
}
__global__ void findConvoyDeviceSelf(History* d_history, int* d_historyMatchSelf)
{
	if(findHistoryMatchSelf(&(d_history[blockIdx.x]),threadIdx.x))
	{
		atomicMin(d_historyMatchSelf, d_history[blockIdx.x].ID);
	}
}

__global__ void insertIndex(History* d_history, int* d_historyMatch, int* d_historyIDs)
{
	int index = blockIdx.x*blockDim.x + threadIdx.x;
	if( d_historyMatch[blockIdx.y] == index)
	{
		d_historyIDs[blockIdx.y] = d_history[blockIdx.x].ID;
	}
}

__global__ void memSetHistoryMatch(int* d_historyMatch)
{
	d_historyMatch[threadIdx.x] = INT_MAX;
}

__global__ void update(PointCellDevice* d_interval)
{

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
	cudaStream_t stream2, stream3;
	cudaStreamCreate(&stream2);
	cudaStreamCreate(&stream3);
	cudaEventCreate(&startEvent);
	cudaEventCreate(&stopEvent);
	cudaEventCreate(&start2Event);
	cudaEventCreate(&stop2Event);
	cudaEventRecord(startEvent, 0);
	ConvoyTracker tracker;
	std::vector<PointCellDevice> vehicles;
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

		if(tracker.historySize > 0)
		{
			compensateEgoMotionHistory<<<tracker.historySize, MAX_LENGTH_HIST_CONV>>>(tracker.d_history_ptr, deltaX, deltaY, deltaYaw);
			vehicles = tracker.reader.processLaserData(number,tracker.getCurrentSpeed(), tracker.getCurrentYawRate());
			if(tracker.convoySize > 0)
			{
				compensateEgoMotionConvoy<<<tracker.convoySize, MAX_LENGTH_HIST_CONV,0, stream2>>>(tracker.d_convoys_ptr, deltaX, deltaY, deltaYaw);
			}
			if(tracker.intervalSize > 0)
			{
				compensateEgoMotionMap<<<1,tracker.intervalSize,0,stream3>>>(tracker.d_intervalMap_ptr, tracker.d_subIntvl_ptr, deltaX, deltaY, deltaYaw);
			}
			cudaStreamSynchronize(stream3);
			for(uint j=0; j<tracker.intervalSize;j++)
			{
				trackedVehicles.push_back(&(tracker.h_intervalMap[j]));
			}
		//	tracker.shiftStructure(deltaX);
		//	tracker.rotateStructure(deltaYaw, deltaY);
			*tracker.h_historyMatchSelf = INT_MAX;
			findConvoyDeviceSelf<<<tracker.historySize, MAX_LENGTH_HIST_CONV>>>(tracker.d_history_ptr, tracker.d_historyMatchSelf_ptr);

			//2. Predict current vehicle states
		/*	for(uint j = 0; j < tracker.intervalSize; j++)
			{
				tracker.h_intervalMap[j].predict();
				trackedVehicles.push_back(&tracker.h_intervalMap[j]);
			}*/
			cudaDeviceSynchronize();
			if(*tracker.h_historyMatchSelf != INT_MAX)
			{
				tracker.findConvoySelf(*tracker.h_historyMatchSelf);
			}
		}
		else
		{
			if(tracker.intervalSize > 0)
			{
				compensateEgoMotionMap<<<1,tracker.intervalSize>>>(tracker.d_intervalMap_ptr, tracker.d_subIntvl_ptr, deltaX, deltaY, deltaYaw);
			}
			vehicles = tracker.reader.processLaserData(number,tracker.getCurrentSpeed(), tracker.getCurrentYawRate());
			cudaDeviceSynchronize();
			for(uint j=0; j<tracker.intervalSize;j++)
			{
				trackedVehicles.push_back(&(tracker.h_intervalMap[j]));
			}

		/*	tracker.shiftStructure(deltaX);
			tracker.rotateStructure(deltaYaw, deltaY);

			//2. Predict current vehicle states
			for(uint j = 0; j < tracker.intervalSize; j++)
			{
				tracker.h_intervalMap[j].predict();
				trackedVehicles.push_back(&tracker.h_intervalMap[j]);
			}*/
		}
		tracker.transformDataFromDevice();

		//3. Associate and Update
		tracker.associateAndUpdate(vehicles, trackedVehicles);
		cudaEventRecord(stop2Event, 0);
		cudaEventSynchronize(stop2Event);
		 float time;
		 cudaEventElapsedTime(&time, start2Event, stop2Event);
		 compensateHistory[i] = time;
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
	 float time;
	 cudaEventElapsedTime(&time, startEvent, stopEvent);
#ifdef PRINT
	 std::cout << "Overall Time: " << time << std::endl;
#else
	 std::cout << time << std::endl;
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
void ConvoyTracker::associateAndUpdate(std::vector<PointCellDevice> vehicles, std::vector<PointCellDevice*> trackedVehicles)
{
	//initialize all IDs in possible history to -1 to have no false detection in findConvoy
	memSetHistoryMatch<<<1,MAX_SEGMENTS>>>(d_historyMatch_ptr);
	convoyCheckSize = 0;
	std::vector<int> indicesToAdd;
//	std::vector<PointCellDevice> convoyCheck;
	for(uint i = 0; i<vehicles.size(); i++)
	{
		double x = vehicles.at(i).getX();
		double y = vehicles.at(i).getY();
		double theta = vehicles.at(i).getTheta();

		double minDist = INT_MAX;
		double minIndex = INT_MAX;
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

			//only accept current vehicle as possible match if the majority of the distance is caused by the change of the x value
			//that´s because cars normally are moving more in x direction instead of y
			if(dist < minDist) //&& fabs(y-y1) < ASSOCIATION_Y_THRESHOLD)
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
			vehicles.at(i).setID(ID);
			indicesToAdd.push_back(i);
			history[endIndexHistory].ID = ID;
			history[endIndexHistory].tracks[0].subIntvl = 0.5;
			history[endIndexHistory].tracks[0].x = vehicles.at(i).getX();
			history[endIndexHistory].tracks[0].y = vehicles.at(i).getY();
			history[endIndexHistory].tracks[0].theta = vehicles.at(i).getTheta();
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
			h_convoyCheck[convoyCheckSize] = vehicles.at(i);
			++convoyCheckSize;
//			findConvoy(vehicles.at(i));
		}
		else
		{
			//vehicle matched, update

			PointCellDevice* tmp = trackedVehicles.at(trackedVehicles.size() -1 );
			PointCellDevice* update = trackedVehicles.at(minIndex);
			int intvl = floor(update->getX());
			intvl += CARINTERVAL;
			update->update(vehicles.at(i).data);
			update->setX((intvl-CARINTERVAL) + 0.5);
		//	findConvoy(*update);
#ifdef PRINT
			std::cout << "Update ID " << update->getID() << std::endl;
#endif
			int historyIndex = findHistoryWithID(update->getID());
#ifdef PRINT
			std::cout << "HistoryIndex " << historyIndex << std::endl;
#endif
			if(checkHistoryForDuplicate((intvl-CARINTERVAL) + 0.5, historyIndex))
			{
				int index = history[historyIndex].endIndex;
				history[historyIndex].tracks[index].subIntvl = 0.5;
				history[historyIndex].tracks[index].x = update->getX();
				history[historyIndex].tracks[index].y = update->getY();
				history[historyIndex].tracks[index].theta = update->getTheta();
				index = (index+1)%MAX_LENGTH_HIST_CONV;
				history[historyIndex].endIndex = index;
				if(index == history[historyIndex].startIndex)
				{
					history[historyIndex].startIndex = (history[historyIndex].startIndex+1)%NUM_HIST;
				}
	//			std::cout << "Added Vehicle to History with Index" << historyIndex  << " HistorySize: "  << historySize << std::endl;
				currentHistoryOnDevice = false;
			}
			trackedVehicles.at(minIndex) = tmp;
			trackedVehicles.pop_back();
#ifdef PRINT
			std::cout << "Updated vehicle with ID " << update->getID() << std::endl;
#endif
			h_convoyCheck[convoyCheckSize] = *update;
			++convoyCheckSize;
	//		std::cout << "ConvoyCheck with ID " << update->getID() << std::endl;
	//		findConvoy(*update);
		}
	}

	for(uint k = 0; k < trackedVehicles.size(); k++)
	{
		PointCellDevice* tmp = trackedVehicles.at(k);
		//delete all tracks that could not be matched
		for(uint m = 0; m < intervalSize; m++)
		{
			/*if(tmp == &intervalMap.at(m))
			{
				intervalMap.at(m) = intervalMap.at(intervalMap.size()-1);
				intervalMap.pop_back();
				break;
			}*/
			if(tmp == &h_intervalMap[m])
			{
				h_intervalMap[m] = h_intervalMap[--intervalSize];
				break;
			}
		}
	}

	for(uint k = 0; k < indicesToAdd.size(); k++)
	{
	//	intervalMap.push_back(vehicles.at(indicesToAdd.at(k)));
		if(intervalSize < MAX_SEGMENTS)
		{
			h_intervalMap[intervalSize++] = vehicles.at(indicesToAdd.at(k));
		}
	}

	if(historySize > 0 && convoyCheckSize >0)
	{
		dim3 grid(historySize, convoyCheckSize);
		findConvoyDevice<<<grid, MAX_LENGTH_HIST_CONV>>>(d_newVeh_ptr,d_history_ptr,d_historyMatch_ptr);
//		insertIndex<<<grid, MAX_LENGTH_HIST_CONV>>>(d_history_ptr,d_historyMatch_ptr,d_historyIDs_ptr);
		cudaDeviceSynchronize();
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
							convoys[j].tracks[currentConvoy.endIndexTracks].x = interval+0.5;
							convoys[j].tracks[currentConvoy.endIndexTracks].y = vehicle.getY();
							convoys[j].tracks[currentConvoy.endIndexTracks].theta = vehicle.getTheta();
							convoys[j].tracks[currentConvoy.endIndexTracks].subIntvl = 0.5;
							convoys[j].endIndexTracks = index;
							if(index == convoys[j].startIndexTracks)
							{
								convoys[j].startIndexTracks = (convoys[j].startIndexTracks+1)%MAX_LENGTH_HIST_CONV;
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
							convoys[j].tracks[currentConvoy.endIndexTracks].x = interval+0.5;
							convoys[j].tracks[currentConvoy.endIndexTracks].y = vehicle.getY();
							convoys[j].tracks[currentConvoy.endIndexTracks].theta = vehicle.getTheta();
							convoys[j].tracks[currentConvoy.endIndexTracks].subIntvl = 0.5;
							convoys[j].endIndexTracks = index;
							if(index == convoys[j].startIndexTracks)
							{
								convoys[j].startIndexTracks = (convoys[j].startIndexTracks+1)%MAX_LENGTH_HIST_CONV;
							}
						}
						int IDindex = (currentConvoy.endIndexID+1)%MAX_LENGTH_HIST_CONV;
						convoys[j].participatingVehicles[currentConvoy.endIndexID] = vehicle.getID();
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
					else if (it2 != INT_MAX)
					{
						int index = (currentConvoy.endIndexTracks+1)%MAX_LENGTH_HIST_CONV;
						//check if this x value is already contained
						if(checkConvoyForDuplicate(interval+0.5, currentConvoy))
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
					convoys[cIndex].tracks[0].x = interval+0.5;
					convoys[cIndex].tracks[0].y = vehicle.getY();
					convoys[cIndex].tracks[0].theta = vehicle.getTheta();
					convoys[cIndex].tracks[0].subIntvl = 0.5;
					endIndexConvoys = (endIndexConvoys+1)%NUM_CONV;
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

void ConvoyTracker::transformDataFromDevice()
{
	std::vector<int> toDelete;

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
		if(convoys[i].tracks[end].x < -5)
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

/*	toDelete.clear();
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
	}*/
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
