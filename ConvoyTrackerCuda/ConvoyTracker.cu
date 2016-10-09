/*
 * ConvoyTracker.cu
 *
 *  Created on: 06.06.2016
 *      Author: Sebastian Reinhart
 */

#include "ConvoyTracker.cuh"
#include "PointCellDevice.cuh"

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
	xSubInterval = 0;

	/*size_t sizeHist = NUM_HIST;
	sizeHist *= (MAX_LENGTH_HIST_CONV*sizeof(PointCell));
	cudaError_t error = cudaMalloc((void **) &d_history, sizeHist);
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
	}*/
}

ConvoyTracker::~ConvoyTracker() {
	// TODO Auto-generated destructor stub
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

/*
 * data* = [stateVector | stateCopy | F | P | H | R | K | I ]
 */
typedef struct pc
{
	double* data;
	double subIntvl;
	int ID;
}PC;

__device__ void shiftHistory(PointCell* d_pc, double x)
{
	//update history
	d_pc->subInvtl += x;
	int numIntervals = (int) ((d_pc->subInvtl) / INTERVALL_LENGTH);
	d_pc->setX(d_pc->getX() - numIntervals);
	d_pc->subInvtl -= numIntervals;

	//check whether current History is already behind our car
/*	if(it->second.at(it->second.size()-1).getX() < -5)
	{
		//if yes, mark history to delete
		toDelete.push_back(it->first);
	}*/
}

__device__ void shiftConvoys(EMLPos* d_eml, double x)
{
	d_eml->subIntvl += x;
	int numIntervals = (int) ((d_eml->subIntvl) / INTERVALL_LENGTH);
	d_eml->x -= numIntervals;
	d_eml->subIntvl -= numIntervals;
}

__global__ void compensateEgoMotion(PointCell* d_history, EMLPos* d_convoy, double x, double y, double angle)
{
	int index = blockIdx.x*blockDim.x + threadIdx.x;
	shiftHistory(&(d_history[index]), x);
	if(blockIdx.x < 20)
	{
		shiftConvoys(&(d_convoy[index]), x);
	}
}

__global__ void testPointCell(PointCellDevice* d_stateVec)
{
	double a = d_stateVec[threadIdx.x].getX();
	d_stateVec[threadIdx.x].setX(2*a);
	printf("PC in Kernel with X %d\n", d_stateVec[threadIdx.x].getX());
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
		printf("GPU Device %d: \"%s\" with compute capability %d.%d\n\n", devID,
				deviceProp.name, deviceProp.major, deviceProp.minor);
	}

	cudaSetDeviceFlags(cudaDeviceMapHost);

/*	PointCell a[5];
	for(int i=0; i<5; i++)
	{
		a[i].setX(45);
		a[i].setY(10);
		a[i].setTheta(5*M_PI/180.0);
		a[i].setVelocity(30);
		a[i].setPhi(0);
	}

	PointCellDevice b;
	b.setX(45);
	b.setY(10);
	b.setTheta(5*M_PI/180.0);
	b.setVelocity(30);
	b.setPhi(0);

	a[0].predict();
	b.predict();
	std::cout << "PC X " << a[0].getX() << std::endl;
	std::cout << "PC Y " << a[0].getY() << std::endl;
	std::cout << "PC Theta " << a[0].getTheta() << std::endl;
	std::cout << "PC Vel " << a[0].getVelocity() << std::endl;
	std::cout << "PC Phi " << a[0].getPhi() << std::endl;

	std::cout << "PCD X " << b.getX() << std::endl;
	std::cout << "PCD Y " << b.getY() << std::endl;
	std::cout << "PCD Theta " << b.getTheta() << std::endl;
	std::cout << "PCD Vel " << b.getVelocity() << std::endl;
	std::cout << "PCD Phi " << b.getPhi() << std::endl;

	double c[] = {50,10,0,30,0};
	Matrix<double> testM(5,1);
	testM.put(0,0,50);
	testM.put(1,0,10);
	testM.put(2,0,0);
	testM.put(3,0,30);
	testM.put(4,0,0);
	a[0].update(testM);
	b.update(c);
	std::cout << "PC X " << a[0].getX() << std::endl;
	std::cout << "PC Y " << a[0].getY() << std::endl;
	std::cout << "PC Theta " << a[0].getTheta() << std::endl;
	std::cout << "PC Vel " << a[0].getVelocity() << std::endl;
	std::cout << "PC Phi " << a[0].getPhi() << std::endl;

	std::cout << "PCD X " << b.getX() << std::endl;
	std::cout << "PCD Y " << b.getY() << std::endl;
	std::cout << "PCD Theta " << b.getTheta() << std::endl;
	std::cout << "PCD Vel " << b.getVelocity() << std::endl;
	std::cout << "PCD Phi " << b.getPhi() << std::endl;*/
/*	PointCellDevice* d_pc;

	for(int i=0; i<5; i++)
	{
		std::cout << "PC Before on CPU X " << a[i].getX() << std::endl;
	}
	//PointCell* d_pc;
	double* d_matrices;
	std::vector<double*> testVector;
	for(int i=0; i<5; i++)
	{
		double* tmp;
		cudaMalloc((void**) &tmp, 185*sizeof(double));
		testVector.push_back(tmp);
	}
	cudaMalloc((void**) &d_pc, 5*sizeof(PointCellDevice));
	cudaMemcpy(d_pc, a, 5*sizeof(PointCellDevice), cudaMemcpyHostToDevice);
	//cudaMemcpy(d_matrices, a.data, 185*sizeof(double), cudaMemcpyHostToDevice);
	for(int i=0; i<5; i++)
	{
		cudaMemcpy(testVector[i], a[i].data, 185*sizeof(double), cudaMemcpyHostToDevice);
		cudaMemcpy(&((d_pc+i)->data), &testVector[i], sizeof(double*), cudaMemcpyHostToDevice);
	}

//	for(int i=0; i<5; i++)
//	{
	//	d_pc[i].stateCopy.matrix = &(d_matrices[i*10]);
	//	d_pc[0].stateVector.matrix = d_matrices;
//	}
//	for(int i=0; i<5; i++)
//	{
//		cudaMemcpy(d_matrices, a[0].stateCopy.matrix, sizeof(PointCell), cudaMemcpyHostToDevice);
//		cudaMemcpy(&(d_matrices[5]), a[0].stateVector.matrix, sizeof(PointCell), cudaMemcpyHostToDevice);
//	}
	testPointCell<<<1,5>>>(d_pc);
	//cudaMemcpy(&d_matrices, &(d_pc->stateVector.matrix), sizeof(double*), cudaMemcpyDeviceToHost);
//	cudaMemcpy(a.data, d_matrices, 185*sizeof(double), cudaMemcpyDeviceToHost);
	for(int i=0; i<5; i++)
	{
		cudaMemcpy(a[i].data, testVector[i], 185*sizeof(double), cudaMemcpyDeviceToHost);
	}
	//cudaMemcpy(&a, d_pc, sizeof(PointCell), cudaMemcpyDeviceToHost);
	cudaDeviceSynchronize();
	for(int i=0; i<5; i++)
	{
		std::cout << "PC AFTER on CPU X " << a[i].getX() << std::endl;
	}

	cudaMemcpy(d_pc, a, 5*sizeof(PointCellDevice), cudaMemcpyHostToDevice);
	for(int i=0; i<5; i++)
	{
		cudaMemcpy(testVector[i], a[i].data, 185*sizeof(double), cudaMemcpyHostToDevice);
		cudaMemcpy(&((d_pc+i)->data), &testVector[i], sizeof(double*), cudaMemcpyHostToDevice);
	}
	testPointCell<<<1,5>>>(d_pc);
	//cudaMemcpy(&d_matrices, &(d_pc->stateVector.matrix), sizeof(double*), cudaMemcpyDeviceToHost);
//	cudaMemcpy(a.data, d_matrices, 185*sizeof(double), cudaMemcpyDeviceToHost);
	for(int i=0; i<5; i++)
	{
		cudaMemcpy(a[i].data, testVector[i], 185*sizeof(double), cudaMemcpyDeviceToHost);
	}
	//cudaMemcpy(&a, d_pc, sizeof(PointCell), cudaMemcpyDeviceToHost);
	cudaDeviceSynchronize();
	for(int i=0; i<5; i++)
	{
		std::cout << "PC AFTER2 on CPU X " << a[i].getX() << std::endl;
	}*/

	cudaEvent_t startEvent, stopEvent;
	cudaEventCreate(&startEvent);
	cudaEventCreate(&stopEvent);
	cudaEventRecord(startEvent, 0);
	ConvoyTracker tracker;
	std::vector<PointCellDevice> vehicles;
	long dur[NUM_MEASUREMENT];
	long compensateHistory[NUM_MEASUREMENT];
	long compensateData[NUM_MEASUREMENT];

	for(int i=0; i<NUM_MEASUREMENT; i++)
	{
		std::vector<PointCellDevice*> trackedVehicles;
		std::string number = getNextMeasureAsString(i);
		tracker.readEMLData(number);
		auto start = std::chrono::steady_clock::now();
		vehicles = tracker.reader.processLaserData(number,tracker.getCurrentSpeed(), tracker.getCurrentYawRate());
		auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start);
	//	std::cout << "Duration of ConvoyTracking: " << duration.count() << std::endl;
		dur[i] = duration.count();
		//1. Compensate own vehicle motion
		double deltaX = tracker.getX() - tracker.getXOld();
		double deltaY = tracker.getY() - tracker.getYOld();
		double deltaYaw = tracker.getYaw() - tracker.getYawOld();

		start = std::chrono::steady_clock::now();
		tracker.shiftStructure(deltaX);
		tracker.rotateStructure(deltaYaw, deltaY);

		//2. Predict current vehicle states
		for(uint j = 0; j < tracker.intervalMap.size(); j++)
		{
			tracker.intervalMap.at(j).predict();
			trackedVehicles.push_back(&tracker.intervalMap.at(j));
		}
		duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start);
		compensateData[i] = duration.count();

		start = std::chrono::steady_clock::now();
		tracker.shiftConvoyHistory(deltaX);
	//	tracker.transformDataToDevice();
	//	compensateEgoMotion<<<NUM_HIST,MAX_LENGTH_HIST_CONV>>>(tracker.d_history, tracker.d_convoys, deltaX, deltaY, deltaYaw);
	//	tracker.transformDataFromDevice();
		duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start);
		compensateHistory[i] = duration.count();
		tracker.rotateConvoyHistory(deltaYaw, deltaY);
		//3. Associate and Update
		tracker.associateAndUpdate(vehicles, trackedVehicles);

	}
	cudaEventRecord(stopEvent, 0);
	cudaEventSynchronize(stopEvent);
	long sum = 0;
	long sumH = 0;
	long sumD = 0;

	for(int i = 0; i< NUM_MEASUREMENT; i++)
	{
		sum += dur[i];
		sumH += compensateHistory[i];
		sumD += compensateData[i];
	}
	sum /= NUM_MEASUREMENT;
	sumH /= NUM_MEASUREMENT;
	sumD /= NUM_MEASUREMENT;
	std::cout << "Duration of Process laserdata: " << sum << std::endl;
	std::cout << "Duration of compensate Data: " << sumD << std::endl;
	std::cout << "Duration of compensate History: " << sumH << std::endl;

	 float time;
	 cudaEventElapsedTime(&time, startEvent, stopEvent);
	 std::cout << "Overall Time: " << time << std::endl;
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
	std::cout << measurePath.str() << std::endl;
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
	for(uint i = 0; i<vehicles.size(); i++)
	{
		double x = vehicles.at(i).getX();
		double y = vehicles.at(i).getY();
		double theta = vehicles.at(i).getTheta();

		double minDist = INT_MAX;
		double minIndex = INT_MAX;

		std::cout << "X: " << x << " Y: " << y << " Theta: " << theta <<std::endl;
		for(uint j = 0; j<trackedVehicles.size(); j++)
		{
			double x1 = trackedVehicles.at(j)->getX();
			double y1 = trackedVehicles.at(j)->getY();
			double theta1 = trackedVehicles.at(j)->getTheta();

			std::cout << "X1: " << x1 << " Y1: " << y1<< " Theta1: " << theta1 <<std::endl;

			double dist = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1) + (theta - theta1)*(theta - theta1));

			//only accept current vehicle as possible match if the majority of the distance is caused by the change of the x value
			//that´s because cars normally are moving more in x direction instead of y
			if(dist < minDist) //&& fabs(y-y1) < ASSOCIATION_Y_THRESHOLD)
			{
				minDist = dist;
				minIndex = j;
			}
		}
		std::cout << "Min distance: " << minDist << std::endl;
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
			std::cout << "Added new Vehicle with ID " << ID << std::endl;

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
			findConvoy(*update);
			history.at(update->getID()).push_back(*update);

			trackedVehicles.at(minIndex) = tmp;
			trackedVehicles.pop_back();

			std::cout << "Updated vehicle with ID " << update->getID() << std::endl;

			findConvoy(*update);
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
	xSubInterval += xMotion;
	int numIntervals = (int) (xSubInterval / INTERVALL_LENGTH);
	xSubInterval -= numIntervals;
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
				- xSubInterval;
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

void ConvoyTracker::transformDataToDevice()
{
	int counter = 0;
	int index;
	for (std::map<int,std::vector<PointCellDevice> >::iterator it=history.begin(); it!=history.end(); ++it)
	{
		index = counter*MAX_LENGTH_HIST_CONV;
		size_t size = it->second.size()*sizeof(PointCell);
		cudaError_t err = cudaMemcpy(&(d_history[index]), it->second.data(), size, cudaMemcpyHostToDevice);
		if(err != cudaSuccess)
		{
			printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(err), err, __LINE__);
		}
		++counter;
	}
	counter = 0;
	for(uint i = 0; i < convoys.size(); i++)
	{
		index = counter*MAX_LENGTH_HIST_CONV;
		size_t size = convoys.at(i).tracks.size()*sizeof(EMLPos);
		cudaError_t err = cudaMemcpy(&(d_convoys[index]), convoys.at(i).tracks.data(), size, cudaMemcpyHostToDevice);
		if(err != cudaSuccess)
		{
			printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(err), err, __LINE__);
		}
		++counter;
	}
}

void ConvoyTracker::transformDataFromDevice()
{
	std::vector<int> toDelete;
	int counter = 0;
	int index;
	for (std::map<int,std::vector<PointCellDevice> >::iterator it=history.begin(); it!=history.end(); ++it)
	{
		index = counter*MAX_LENGTH_HIST_CONV;
		size_t size = it->second.size()*sizeof(PointCell);
		cudaError_t err = cudaMemcpy(it->second.data(), &(d_history[index]), size, cudaMemcpyDeviceToHost);
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
	}

	toDelete.clear();
	counter = 0;

	for(uint i = 0; i < convoys.size(); i++)
	{
		index = counter*MAX_LENGTH_HIST_CONV;
		size_t size = convoys.at(i).tracks.size()*sizeof(EMLPos);
		cudaError_t err = cudaMemcpy(convoys.at(i).tracks.data(), &(d_convoys[index]), size, cudaMemcpyDeviceToHost);
		if(err != cudaSuccess)
		{
			printf(
				"cudaGetDeviceProperties returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(err), err, __LINE__);
		}
		if(convoys.at(i).tracks.at(convoys.at(i).tracks.size()-1).x < -5)
		{
			toDelete.push_back(i);
		}
		++counter;
	}

	for(uint i=0; i<toDelete.size(); i++)
	{
		Convoy tmp = convoys.at(convoys.size()-1);
		convoys.at(toDelete.at(i)) = tmp;
		convoys.pop_back();
	}
}
