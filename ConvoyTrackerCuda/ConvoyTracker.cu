/*
 * ConvoyTracker.cu
 *
 *  Created on: 06.06.2016
 *      Author: Sebastian Reinhart
 */

#include "ConvoyTracker.cuh"

ConvoyTracker::ConvoyTracker() {
	// TODO Auto-generated constructor stub
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

	for(int i=0; i<MAX_SEGMENTS;i++)
	{
		setMem[i] = INT_MAX;
	}

	size_t sizeHist = NUM_HIST;
	sizeHist *= (MAX_LENGTH_HIST_CONV*sizeof(PointCellDevice));
	cudaError_t error = cudaMalloc((void **) &d_history, sizeHist);
	if(error != cudaSuccess)
	{
		printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_historyMatch, MAX_SEGMENTS*sizeof(int), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_historyMatch, MAX_SEGMENTS*sizeof(int));
	if(error != cudaSuccess)
	{
		printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_historyMatchSelf, MAX_SEGMENTS*sizeof(int), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_historyMatchSelf, MAX_SEGMENTS*sizeof(int));
	if(error != cudaSuccess)
	{
		printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_newVeh, MAX_SEGMENTS*sizeof(PointCellDevice));
	if(error != cudaSuccess)
	{
		printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	size_t sizeConv = NUM_CONV;
	sizeConv *= (MAX_LENGTH_HIST_CONV*sizeof(EMLPos));
	error = cudaMalloc((void **) &d_convoys, sizeConv);
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
}

ConvoyTracker::~ConvoyTracker() {
	// TODO Auto-generated destructor stub
	cudaFreeHost(xSubInterval);
	cudaFreeHost(h_historyMatch);
	cudaFree(d_newVeh);
	cudaFree(d_historyMatch);
	cudaFree(d_history);
	cudaFree(d_convoys);
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


__device__ void shiftRotateHistory(PointCellDevice* d_pc, double x, double y, double theta)
{
	//update history
	d_pc->subInvtl += x;
	int numIntervals = (int) ((d_pc->subInvtl) / INTERVALL_LENGTH);
	d_pc->setX(d_pc->getX() - numIntervals);
	d_pc->subInvtl = d_pc->subInvtl-numIntervals;

	double angleInRadians = theta*M_PI/180.0;
	double mat[2][2] = { { cos(angleInRadians), -sin(angleInRadians) },
			{ sin(angleInRadians), cos(angleInRadians) } };
	//update history
			d_pc->setY(d_pc->getY() - y);
			d_pc->setTheta(d_pc->getTheta() - angleInRadians);

			double xAbs = d_pc->getX();
			double yAbs = d_pc->getY();

			xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
			yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

			d_pc->setY(d_pc->getY() - yAbs);
			d_pc->subInvtl -= xAbs;
}

__device__ void shiftRotateConvoy(EMLPos* d_eml, double x, double y, double theta)
{
	d_eml->subIntvl += x;
	int numIntervals = (int) ((d_eml->subIntvl) / INTERVALL_LENGTH);
	d_eml->x -= numIntervals;
	d_eml->subIntvl -= numIntervals;

	double angleInRadians = theta*M_PI/180.0;
	double mat[2][2] = { { cos(angleInRadians), -sin(angleInRadians) },
			{ sin(angleInRadians), cos(angleInRadians) } };

	d_eml->y -= y;
	d_eml->theta -= angleInRadians;

	double xAbs = d_eml->x;
	double yAbs = d_eml->y;

	xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
	yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

	d_eml->y -= yAbs;
	d_eml->subIntvl -= xAbs;

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

__device__ bool findHistoryMatch(PointCellDevice* trackedVehicles, PointCellDevice* d_history)
{
	bool result = (d_history->getID() != trackedVehicles->getID());
	result = (result && (d_history->getX() - 0.5 <= trackedVehicles->getX()));
	result = (result && (trackedVehicles->getX() <= d_history->getX() + 0.5));
	result = (result && (d_history->getY() - 1.0 <= trackedVehicles->getY()));
	result = (result && (trackedVehicles->getY() <= d_history->getY() + 1.0));
	return result;
}
__device__ bool findHistoryMatchSelf(PointCellDevice* d_history)
{
	bool result = true;
	result = (result && (d_history->getX() - 0.5 <= 0));
	result = (result && (0 <= d_history->getX() + 0.5));
	result = (result && (d_history->getY() - 1.0 <= 0));
	result = (result && (0 <= d_history->getY() + 1.0));
	//result = (result && (index < d_history->endIndex));
	//result = (result && (index >= d_history->startIndex));
	return result;
}
__global__ void compensateEgoMotionMap(PointCellDevice* d_interval, double* d_subIntvl, double x, double y, double angle)
{
	int index = blockIdx.x*blockDim.x + threadIdx.x;
	computeIntervalMap(&(d_interval[index]), x, y, angle, d_subIntvl);
}
__global__ void compensateEgoMotionHistory(PointCellDevice* d_history, double x, double y, double angle)
{
	int index = blockIdx.x*blockDim.x + threadIdx.x;
	shiftRotateHistory(&(d_history[index]), x, y, angle);
}

__global__ void compensateEgoMotionConvoy(EMLPos* d_convoy, double x, double y, double angle)
{
	int index = blockIdx.x*blockDim.x + threadIdx.x;
	shiftRotateConvoy(&(d_convoy[index]), x, y, angle);
}

__global__ void compensateEgoMotion(PointCellDevice* d_history, EMLPos* d_convoy, PointCellDevice* d_interval, double* d_subIntvl, double x, double y, double angle, int numConv, int intvlSize)
{
	int index = blockIdx.x*blockDim.x + threadIdx.x;
	shiftRotateHistory(&(d_history[index]), x, y, angle);
	if(blockIdx.x < numConv)
	{
		shiftRotateConvoy(&(d_convoy[index]), x, y, angle);
	}
	if(blockIdx.x < intvlSize)
	{
		if(threadIdx.x == 0)
		{
			computeIntervalMap(&(d_interval[blockIdx.x]), x, y, angle, d_subIntvl);

		}
	}
}

__global__ void findConvoyDevice(PointCellDevice* trackedVehicles, PointCellDevice* d_history, int* d_historyMatch)
{
	int index = blockIdx.x*blockDim.x + threadIdx.x;
	if(findHistoryMatch(&(trackedVehicles[blockIdx.y]),&(d_history[index])))
	{
#ifdef PRINT
		printf("TrackedID %d, HistoryID %d\n", trackedVehicles[blockIdx.y].getID(), d_history[index].getID());
#endif
		atomicMin(&(d_historyMatch[blockIdx.y]), d_history[index].getID());
	}
}

__global__ void findConvoyDeviceSelf(PointCellDevice* d_history, int* d_historyMatchSelf)
{
	int index = blockIdx.x*blockDim.x + threadIdx.x;
	if(findHistoryMatchSelf(&(d_history[index])))
	{
		atomicMin(d_historyMatchSelf, d_history[index].getID());
	}
}

__global__ void insertIndex(PointCellDevice* d_history, int* d_historyMatch)
{
	int index = blockIdx.x*blockDim.x + threadIdx.x;
	if(d_historyMatch[blockIdx.y] == index)
	{
		d_historyMatch[blockIdx.y] = d_history[index].getID();
	}
}

__global__ void memSetHistory(PointCellDevice* d_history)
{
	int index = blockIdx.x*blockDim.x + threadIdx.x;
//	d_history[index].setID(-1);
	d_history[index].setX(-10);
//	d_history[index].setY(-10);
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

	cudaEvent_t startEvent, stopEvent, start2Event, stop2Event;
	cudaEventCreate(&startEvent);
	cudaEventCreate(&stopEvent);
	cudaEventCreate(&start2Event);
	cudaEventCreate(&stop2Event);
	cudaEventRecord(startEvent, 0);
	ConvoyTracker tracker;
	memSetHistory<<<NUM_HIST, MAX_LENGTH_HIST_CONV>>>(tracker.d_history);
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

		tracker.transformDataToDevice();
		if(tracker.history.size() > 0)
		{
			compensateEgoMotionHistory<<<tracker.history.size(), MAX_LENGTH_HIST_CONV>>>(tracker.d_history, deltaX, deltaY, deltaYaw);
			vehicles = tracker.reader.processLaserData(number,tracker.getCurrentSpeed(), tracker.getCurrentYawRate());
			if(tracker.convoys.size() > 0)
			{
				compensateEgoMotionConvoy<<<tracker.convoys.size(), MAX_LENGTH_HIST_CONV>>>(tracker.d_convoys, deltaX, deltaY, deltaYaw);
			}
			tracker.shiftStructure(deltaX);
			tracker.rotateStructure(deltaYaw, deltaY);

			*tracker.h_historyMatchSelf = INT_MAX;
			cudaMemcpy(tracker.d_historyMatchSelf, tracker.h_historyMatchSelf, sizeof(int), cudaMemcpyHostToDevice);
			findConvoyDeviceSelf<<<tracker.history.size(), MAX_LENGTH_HIST_CONV>>>(tracker.d_history, tracker.d_historyMatchSelf);
			//2. Predict current vehicle states
			for(uint j = 0; j < tracker.intervalMap.size(); j++)
			{
				tracker.intervalMap.at(j).predict();
				trackedVehicles.push_back(&tracker.intervalMap.at(j));
			}
		}
		else
		{
			vehicles = tracker.reader.processLaserData(number,tracker.getCurrentSpeed(), tracker.getCurrentYawRate());

			tracker.shiftStructure(deltaX);
			tracker.rotateStructure(deltaYaw, deltaY);

			//2. Predict current vehicle states
			for(uint j = 0; j < tracker.intervalMap.size(); j++)
			{
				tracker.intervalMap.at(j).predict();
				trackedVehicles.push_back(&tracker.intervalMap.at(j));
			}
		}

		tracker.transformDataFromDevice();
		cudaMemcpy(tracker.h_historyMatchSelf, tracker.d_historyMatchSelf, sizeof(int), cudaMemcpyDeviceToHost);
		if(*tracker.h_historyMatchSelf != INT_MAX)
		{
			tracker.findConvoySelf(*tracker.h_historyMatchSelf);
		}

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
    	Pos curPos;
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
	xOld = old;
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
	std::vector<int> indicesToAdd;
	std::vector<PointCellDevice> convoyCheck;
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
	//		intervalMap.push_back(vehicles.at(i));
			indicesToAdd.push_back(i);
			std::vector<PointCellDevice> newHist;
			newHist.push_back(vehicles.at(i));
			history.insert(std::pair<int, std::vector<PointCellDevice> > (ID, newHist));
#ifdef PRINT
			std::cout << "Added new Vehicle with ID " << ID << std::endl;
#endif
			currentHistoryOnDevice = false;
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
			if(checkHistoryForDuplicate((intvl-CARINTERVAL) + 0.5, history.at(update->getID())))
			{
				history.at(update->getID()).push_back(*update);
				currentHistoryOnDevice = false;
			}
			if(history.at(update->getID()).size() > MAX_LENGTH_HIST_CONV)
			{
				history.at(update->getID()).erase(history.at(update->getID()).begin());
			}
			trackedVehicles.at(minIndex) = tmp;
			trackedVehicles.pop_back();
#ifdef PRINT
			std::cout << "Updated vehicle with ID " << update->getID() << std::endl;
#endif
			convoyCheck.push_back(*update);
	//		std::cout << "ConvoyCheck with ID " << update->getID() << std::endl;
	//		findConvoy(*update);
		}
	}

	for(uint k = 0; k < trackedVehicles.size(); k++)
	{
		PointCellDevice* tmp = trackedVehicles.at(k);
		//delete all tracks that could not be matched
		for(uint m = 0; m < intervalMap.size(); m++)
		{
			if(tmp == &intervalMap.at(m))
			{
				intervalMap.at(m) = intervalMap.at(intervalMap.size()-1);
				intervalMap.pop_back();
				break;
			}
		}
	}

	for(uint k = 0; k < indicesToAdd.size(); k++)
	{
		intervalMap.push_back(vehicles.at(indicesToAdd.at(k)));
	}

	if(history.size() > 0 && convoyCheck.size() >0)
	{
		cudaError_t err;
		int counter = 0;
		int index;
		size_t size;
		std::vector<int> toDelete;
		//initialize all IDs in possible history to -1 to have no false detection in findConvoy
		memSetHistory<<<NUM_HIST, MAX_LENGTH_HIST_CONV>>>(d_history);
//		if(!currentHistoryOnDevice)
		{
			for (std::map<int,std::vector<PointCellDevice> >::iterator it=history.begin(); it!=history.end(); ++it)
			{
				index = counter*MAX_LENGTH_HIST_CONV;
				size = it->second.size()*sizeof(PointCellDevice);
				err = cudaMemcpy(&(d_history[index]), it->second.data(), size, cudaMemcpyHostToDevice);
				if(err != cudaSuccess)
				{
					printf(
							"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
							cudaGetErrorString(err), err, __LINE__);
				}
				++counter;
			}
			currentHistoryOnDevice = true;
		}
		err = cudaMemcpy(d_newVeh, convoyCheck.data(), convoyCheck.size()*sizeof(PointCellDevice), cudaMemcpyHostToDevice);
		if(err != cudaSuccess)
		{
			printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(err), err, __LINE__);
		}

		err = cudaMemcpy(d_historyMatch,setMem, convoyCheck.size()*sizeof(int), cudaMemcpyHostToDevice);
		if(err != cudaSuccess)
		{
			printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(err), err, __LINE__);
		}
		dim3 grid(history.size(), convoyCheck.size());
		findConvoyDevice<<<grid, MAX_LENGTH_HIST_CONV>>>(d_newVeh,d_history,d_historyMatch);
//		insertIndex<<<grid, MAX_LENGTH_HIST_CONV>>>(d_history,d_historyMatch);
		err = cudaMemcpy(h_historyMatch,d_historyMatch, convoyCheck.size()*sizeof(int), cudaMemcpyDeviceToHost);
		if(err != cudaSuccess)
		{
			printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(err), err, __LINE__);
		}

		for(uint i=0; i<convoyCheck.size();i++)
		{
#ifdef PRINT
			std::cout << "ID " << h_historyMatch[i]  << std::endl;
#endif
			if(h_historyMatch[i] != INT_MAX)
			{
				PointCellDevice vehicle = convoyCheck.at(i);
				double x = vehicle.getX();
				int interval = floor(x);
				int id1 = h_historyMatch[i];
				int id2 = vehicle.getID();
#ifdef PRINT
				std::cout << "ID1 " << id1  << " ID2 " << id2 << std::endl;
#endif
				bool convoyFound = false;
				for(uint j = 0; j < convoys.size(); j++)
				{
					std::vector<int>::iterator it1, it2;
					Convoy currentConvoy = convoys.at(j);
					it1 = std::find(currentConvoy.participatingVehicles.begin(), currentConvoy.participatingVehicles.end(), id1);
					it2 = std::find(currentConvoy.participatingVehicles.begin(), currentConvoy.participatingVehicles.end(), id2);
					if(it1 != currentConvoy.participatingVehicles.end() && it2 != currentConvoy.participatingVehicles.end())
					{
						//convoy already exists with both IDS
						//check if this x value is already contained
						if(checkConvoyForDuplicate(interval+0.5, currentConvoy))
						{
							//x value is not contained
							EMLPos newPos;
							newPos.x = interval+0.5;
							newPos.y = vehicle.getY();
							newPos.theta = vehicle.getTheta();
							newPos.subIntvl = 0.5;
							convoys.at(j).tracks.push_back(newPos);
						}
						if(convoys.at(j).tracks.size() > MAX_LENGTH_HIST_CONV)
						{
							convoys.at(j).tracks.erase(convoys.at(j).tracks.begin());
						}
						convoyFound = true;
						break;
					}
					else if (it1 != currentConvoy.participatingVehicles.end())
					{
						//check if this x value is already contained
						if(checkConvoyForDuplicate(interval+0.5, currentConvoy))
						{
							EMLPos newPos;
							newPos.x = interval+0.5;
							newPos.y = vehicle.getY();
							newPos.theta = vehicle.getTheta();
							newPos.subIntvl = 0.5;
							convoys.at(j).tracks.push_back(newPos);
						}
						if(convoys.at(j).tracks.size() > MAX_LENGTH_HIST_CONV)
						{
							convoys.at(j).tracks.erase(convoys.at(j).tracks.begin());
						}
						convoys.at(j).participatingVehicles.push_back(vehicle.getID());
						if(convoys.at(j).participatingVehicles.size() > MAX_LENGTH_HIST_CONV)
						{
							convoys.at(j).participatingVehicles.erase(convoys.at(j).participatingVehicles.begin());
						}
						convoyFound = true;
						break;
					}
					else if (it2 != currentConvoy.participatingVehicles.end())
					{
						//check if this x value is already contained
						if(checkConvoyForDuplicate(interval+0.5, currentConvoy))
						{
							EMLPos newPos;
							newPos.x = interval+0.5;
							newPos.y = vehicle.getY();
							newPos.theta = vehicle.getTheta();
							newPos.subIntvl = 0.5;
							convoys.at(j).tracks.push_back(newPos);
						}
						if(convoys.at(j).tracks.size() > MAX_LENGTH_HIST_CONV)
						{
							convoys.at(j).tracks.erase(convoys.at(j).tracks.begin());
						}
						convoys.at(j).participatingVehicles.push_back(id1);
						if(convoys.at(j).participatingVehicles.size() > MAX_LENGTH_HIST_CONV)
						{
							convoys.at(j).participatingVehicles.erase(convoys.at(j).participatingVehicles.begin());
						}
						convoyFound = true;
						break;
					}
				}

				if(!convoyFound)
				{
					Convoy newConvoy;
					newConvoy.ID = convoyID++;
					newConvoy.participatingVehicles.push_back(id1);
					newConvoy.participatingVehicles.push_back(id2);
					EMLPos newPos;
					newPos.x = interval+0.5;
					newPos.y = vehicle.getY();
					newPos.theta = vehicle.getTheta();
					newPos.subIntvl = 0.5;
					newConvoy.tracks.push_back(newPos);
					convoys.push_back(newConvoy);
				}
				currentConvoyOnDevice = false;
			}
		}
	}
}

void ConvoyTracker::findConvoy(PointCellDevice vehicle)
{
	double x = vehicle.getX();
	int interval = floor(x);
	for (std::map<int,std::vector<PointCellDevice> >::iterator it=history.begin(); it!=history.end(); ++it)
	{
		if(it->first == vehicle.getID())
		{
			continue;
		}

		for(uint i = 0; i < it->second.size(); i++)
		{
			PointCellDevice pc = it->second.at(i);
			if(pc.getX() - 0.5 <= vehicle.getX() && vehicle.getX() <= pc.getX() + 0.5)
			{
				if(pc.getY() - 1.0 <= vehicle.getY() && vehicle.getY() <= pc.getY() + 1.0)
				{
					//current vehicle position matches with one from history -> add this pointcell to convoy
					int id1 = pc.getID();
					int id2 = vehicle.getID();
					bool convoyFound = false;
					for(uint j = 0; j < convoys.size(); j++)
					{
						std::vector<int>::iterator it1, it2;
						Convoy currentConvoy = convoys.at(j);
						it1 = std::find(currentConvoy.participatingVehicles.begin(), currentConvoy.participatingVehicles.end(), id1);
						it2 = std::find(currentConvoy.participatingVehicles.begin(), currentConvoy.participatingVehicles.end(), id2);
						if(it1 != currentConvoy.participatingVehicles.end() && it2 != currentConvoy.participatingVehicles.end())
						{
							//convoy already exists with both IDS
							//check if this x value is already contained
							if(checkConvoyForDuplicate(interval+0.5, currentConvoy))
							{
								//x value is not contained
								EMLPos newPos;
								newPos.x = interval+0.5;
								newPos.y = vehicle.getY();
								newPos.theta = vehicle.getTheta();
								newPos.subIntvl = 0.5;
								convoys.at(j).tracks.push_back(newPos);
							}
							convoyFound = true;
							break;
						}
						else if (it1 != currentConvoy.participatingVehicles.end())
						{
							//check if this x value is already contained
							if(checkConvoyForDuplicate(interval+0.5, currentConvoy))
							{
								EMLPos newPos;
								newPos.x = interval+0.5;
								newPos.y = vehicle.getY();
								newPos.theta = vehicle.getTheta();
								newPos.subIntvl = 0.5;
								convoys.at(j).tracks.push_back(newPos);
							}
							currentConvoy.participatingVehicles.push_back(vehicle.getID());
							convoyFound = true;
							break;
						}
						else if (it2 != currentConvoy.participatingVehicles.end())
						{
							//check if this x value is already contained
							if(checkConvoyForDuplicate(interval+0.5, currentConvoy))
							{
								EMLPos newPos;
								newPos.x = interval+0.5;
								newPos.y = vehicle.getY();
								newPos.theta = vehicle.getTheta();
								newPos.subIntvl = 0.5;
								convoys.at(j).tracks.push_back(newPos);
							}
							currentConvoy.participatingVehicles.push_back(pc.getID());
							convoyFound = true;
							break;
						}
					}

					if(!convoyFound)
					{
						Convoy newConvoy;
						newConvoy.ID = convoyID++;
						newConvoy.participatingVehicles.push_back(pc.getID());
						newConvoy.participatingVehicles.push_back(vehicle.getID());
						EMLPos newPos;
						newPos.x = interval+0.5;
						newPos.y = vehicle.getY();
						newPos.theta = vehicle.getTheta();
						newPos.subIntvl = 0.5;
						newConvoy.tracks.push_back(newPos);
						convoys.push_back(newConvoy);
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
	for (std::map<int,std::vector<PointCellDevice> >::iterator it=history.begin(); it!=history.end(); ++it)
	{
		for(uint i = 0; i < it->second.size(); i++)
		{
			it->second.at(i).subInvtl += x;
			int numIntervals = (int) ((it->second.at(i).subInvtl) / INTERVALL_LENGTH);
			it->second.at(i).setX(it->second.at(i).getX() - numIntervals);
			it->second.at(i).subInvtl -= numIntervals;
		}

		//check whether current History is already behind our car
		if(it->second.at(it->second.size()-1).getX() < -5)
		{
			//if yes, mark history to delete
			toDelete.push_back(it->first);
		}
	}

	for(uint i=0; i<toDelete.size(); i++)
	{
		history.erase(toDelete.at(i));
	}

	toDelete.clear();

	//update Convoys
	for(uint i = 0; i < convoys.size(); i++)
	{
		for(uint j = 0; j < convoys.at(i).tracks.size(); j++)
		{
			convoys.at(i).tracks.at(j).subIntvl += x;
			int numIntervals = (int) ((convoys.at(i).tracks.at(j).subIntvl) / INTERVALL_LENGTH);
			convoys.at(i).tracks.at(j).x -= numIntervals;
			convoys.at(i).tracks.at(j).subIntvl -= numIntervals;
		}

		if(convoys.at(i).tracks.at(convoys.at(i).tracks.size()-1).x < -5)
		{
			toDelete.push_back(i);
		}
	}

	for(uint i=0; i<toDelete.size(); i++)
	{
		Convoy tmp = convoys.at(convoys.size()-1);
		convoys.at(toDelete.at(i)) = tmp;
		convoys.pop_back();
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
	for (std::map<int,std::vector<PointCellDevice> >::iterator it=history.begin(); it!=history.end(); ++it)
	{
		for(uint i = 0; i < it->second.size(); i++)
		{
			it->second.at(i).setY(it->second.at(i).getY() - y);
			it->second.at(i).setTheta(it->second.at(i).getTheta() - angleInRadians);

			double xAbs = it->second.at(i).getX();
			double yAbs = it->second.at(i).getY();

			xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
			yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

			it->second.at(i).setY(it->second.at(i).getY() - yAbs);
			it->second.at(i).subInvtl -= xAbs;
		}
	}

	//update Convoys
	for(uint i = 0; i < convoys.size(); i++)
	{
		for(uint j = 0; j < convoys.at(i).tracks.size(); j++)
		{
			convoys.at(i).tracks.at(j).y -= y;
			convoys.at(i).tracks.at(j).theta -= angleInRadians;

			double xAbs = convoys.at(i).tracks.at(j).x;
			double yAbs = convoys.at(i).tracks.at(j).y;

			xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
			yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

			convoys.at(i).tracks.at(j).y -= yAbs;
			convoys.at(i).tracks.at(j).subIntvl -= xAbs;
		}
	}
}

void ConvoyTracker::visualizeConvoys()
{
	visualizer.visualizeConvoys(EML, convoys);
}

void ConvoyTracker::visualizeHistory()
{
	visualizer.visualizeHistory(EML, history);
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
		for(uint j = 0; j < intervalMap.size(); j++)
		{
			double x = intervalMap.at(j).getX();
			int interval = floor(x) + CARINTERVAL;
			if(interval == 0)
			{
				//delete content
				toDelete.push_back(j);
				continue;
			}
			intervalMap.at(j).setX(floor(x) - 0.5);
		}

		for(uint j = 0; j < toDelete.size(); j++)
		{
			PointCellDevice tmp = intervalMap.at(intervalMap.size() -1);
			intervalMap.at(intervalMap.size() -1) = intervalMap.at(j);
			intervalMap.at(j) = tmp;
			intervalMap.pop_back();
		}
	}

}
/*
 * rotates the whole map by @param angle and shifts the PCs by @param yMotion
 */
void ConvoyTracker::rotateStructure(double angle, double yMotion) {
	//map for temporary storage of PC that should be moved one interval up
	double angleInRadians = angle * M_PI / 180;

	for(uint i = 0; i < intervalMap.size(); i++)
	{
		PointCellDevice currentVehicle = intervalMap.at(i);
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
		intervalMap.at(i) = currentVehicle;
	}
}

bool ConvoyTracker::checkConvoyForDuplicate(double x, Convoy c)
{
	for(uint i=0; i<c.tracks.size(); i++)
	{
		if(c.tracks.at(i).x == x)
		{
			return false;
		}
	}
	return true;
}

bool ConvoyTracker::checkHistoryForDuplicate(double x, std::vector<PointCellDevice> c)
{
	for(uint i=0; i<c.size(); i++)
	{
		if(c.at(i).getX() == x)
		{
			return false;
		}
	}
	return true;
}

void ConvoyTracker::transformDataToDevice()
{
	int counter = 0;
	int index;
	size_t size;
	cudaError_t err;
	if(!currentHistoryOnDevice)
	{
		for (std::map<int,std::vector<PointCellDevice> >::iterator it=history.begin(); it!=history.end(); ++it)
		{
			index = counter*MAX_LENGTH_HIST_CONV;
			size = it->second.size()*sizeof(PointCellDevice);
			err = cudaMemcpy(&(d_history[index]), it->second.data(), size, cudaMemcpyHostToDevice);
			if(err != cudaSuccess)
			{
				printf(
					"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
					cudaGetErrorString(err), err, __LINE__);
			}
			++counter;
		}
		currentHistoryOnDevice = true;
	}
	if(!currentConvoyOnDevice)
	{
		for(uint i = 0; i < convoys.size(); i++)
		{
			index = i*MAX_LENGTH_HIST_CONV;
			size = convoys.at(i).tracks.size()*sizeof(EMLPos);
			err = cudaMemcpy(&(d_convoys[index]), convoys.at(i).tracks.data(), size, cudaMemcpyHostToDevice);
			if(err != cudaSuccess)
			{
				printf(
					"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
					cudaGetErrorString(err), err, __LINE__);
			}
		}
		currentConvoyOnDevice = true;
	}
	/*size = intervalMap.size()*sizeof(PointCellDevice);
	err = cudaMemcpy(d_intervalMap, intervalMap.data(), size, cudaMemcpyHostToDevice);
	if(err != cudaSuccess)
	{
		printf(
			"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
			cudaGetErrorString(err), err, __LINE__);
	}*/
}

void ConvoyTracker::transformDataFromDevice()
{
	std::vector<int> toDelete;
	int counter = 0;
	int index;
	size_t size;
	cudaError_t err;
	for (std::map<int,std::vector<PointCellDevice> >::iterator it=history.begin(); it!=history.end(); ++it)
	{
		index = counter*MAX_LENGTH_HIST_CONV;
		size = it->second.size()*sizeof(PointCellDevice);
		err = cudaMemcpy(it->second.data(), &(d_history[index]), size, cudaMemcpyDeviceToHost);
		if(err != cudaSuccess)
		{
			printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(err), err, __LINE__);
		}

		//check whether current History is already behind our car
		if(it->second.at(it->second.size()-1).getX() < -5)
		{
			//if yes, mark history to delete
			toDelete.push_back(it->first);
		}
		++counter;
	}

	for(uint i=0; i<toDelete.size(); i++)
	{
		history.erase(toDelete.at(i));
		currentHistoryOnDevice = false;
	}

	toDelete.clear();
	counter = 0;

	for(uint i = 0; i < convoys.size(); i++)
	{
		index = counter*MAX_LENGTH_HIST_CONV;
		size = convoys.at(i).tracks.size()*sizeof(EMLPos);
		err = cudaMemcpy(convoys.at(i).tracks.data(), &(d_convoys[index]), size, cudaMemcpyDeviceToHost);
		if(err != cudaSuccess)
		{
			printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(err), err, __LINE__);
		}
		if(convoys.at(i).tracks.at(convoys.at(i).tracks.size()-1).x < -5)
		{
			toDelete.push_back(i);
			currentConvoyOnDevice = false;
		}
		++counter;
	}

	if(toDelete.size() > 0)
	{
		for(int i=toDelete.size()-1; i >= 0; i--)
		{
			if(toDelete.at(i) != convoys.size()-1)
			{
				convoys.at(toDelete.at(i)) = convoys.at(convoys.size()-1);
			}
			convoys.pop_back();
		}
	}
}

void ConvoyTracker::findConvoySelf(int ID)
{
	double x = 0;
					int interval = floor(x);
					int id1 = -1;
					int id2 = ID;
					std::cout << ID << std::endl;
	#ifdef PRINT
					std::cout << "ID1 " << id1  << " ID2 " << id2 << std::endl;
	#endif
					bool convoyFound = false;
					for(uint j = 0; j < convoys.size(); j++)
					{
						std::vector<int>::iterator it1, it2;
						Convoy currentConvoy = convoys.at(j);
						it1 = std::find(currentConvoy.participatingVehicles.begin(), currentConvoy.participatingVehicles.end(), id1);
						it2 = std::find(currentConvoy.participatingVehicles.begin(), currentConvoy.participatingVehicles.end(), id2);
						if(it1 != currentConvoy.participatingVehicles.end() && it2 != currentConvoy.participatingVehicles.end())
						{
							//convoy already exists with both IDS
							//check if this x value is already contained
							if(checkConvoyForDuplicate(interval+0.5, currentConvoy))
							{
								//x value is not contained
								EMLPos newPos;
								newPos.x = 0.5;
								newPos.y = 0;
								newPos.theta = 0;
								newPos.subIntvl = 0.5;
								convoys.at(j).tracks.push_back(newPos);
							}
							if(convoys.at(j).tracks.size() > MAX_LENGTH_HIST_CONV)
							{
								convoys.at(j).tracks.erase(convoys.at(j).tracks.begin());
							}
							convoyFound = true;
							break;
						}
						else if (it1 != currentConvoy.participatingVehicles.end())
						{
							//check if this x value is already contained
							if(checkConvoyForDuplicate(interval+0.5, currentConvoy))
							{
								EMLPos newPos;
								newPos.x = 0.5;
								newPos.y = 0;
								newPos.theta = 0;
								newPos.subIntvl = 0.5;
								convoys.at(j).tracks.push_back(newPos);
							}
							if(convoys.at(j).tracks.size() > MAX_LENGTH_HIST_CONV)
							{
								convoys.at(j).tracks.erase(convoys.at(j).tracks.begin());
							}
							convoys.at(j).participatingVehicles.push_back(id2);
							if(convoys.at(j).participatingVehicles.size() > MAX_LENGTH_HIST_CONV)
							{
								convoys.at(j).participatingVehicles.erase(convoys.at(j).participatingVehicles.begin());
							}
							convoyFound = true;
							break;
						}
					}

					if(!convoyFound)
					{
						Convoy newConvoy;
						newConvoy.ID = convoyID++;
						newConvoy.participatingVehicles.push_back(id1);
						newConvoy.participatingVehicles.push_back(id2);
						EMLPos newPos;
						newPos.x = interval+0.5;
						newPos.y = 0;
						newPos.theta = 0;
						newPos.subIntvl = 0.5;
						newConvoy.tracks.push_back(newPos);
						convoys.push_back(newConvoy);
					}
					currentConvoyOnDevice = false;
				}

