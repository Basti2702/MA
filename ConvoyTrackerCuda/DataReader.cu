/*
 * DataReader.cpp
 *
 *  Created on: 06.06.2016
 *      Author: Sebastian Reinhart
 */

#include "DataReader.cuh"


DataReader::DataReader() {
	cudaError_t error;
	cudaStreamCreate(&stream1);
	cudaStreamCreate(&stream0);
	error = cudaHostAlloc((void**) &h_data, NUMBER_LASERRAYS*sizeof(laserdata_raw), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void**) &d_data, NUMBER_LASERRAYS*sizeof(laserdata_raw));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &dist, NUMBER_LASERRAYS*sizeof(float), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void**) &d_dist, NUMBER_LASERRAYS*sizeof(float));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_relMeas, MAX_SEGMENTS*3*sizeof(laserdata_cartesian), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void**) &d_relMeas, MAX_SEGMENTS*3*sizeof(laserdata_cartesian));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &thresh, NUMBER_LASERRAYS*sizeof(float), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void**) &d_thresh, NUMBER_LASERRAYS*sizeof(float));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &raw_segments, (MAX_SEGMENTS+1)*sizeof(raw_segment), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void**) &d_rawSegs, (MAX_SEGMENTS+1)*sizeof(raw_segment));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &car_segments, (MAX_SEGMENTS+1)*sizeof(cartesian_segment), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void**) &d_carSegs, (MAX_SEGMENTS+1)*sizeof(cartesian_segment));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error =cudaHostAlloc((void **) &h_minDistance, MAX_SEGMENTS*sizeof(unsigned long long), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void**) &d_minDistance, MAX_SEGMENTS*sizeof(unsigned long long));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}
}

DataReader::~DataReader() {
	// Free device memory
	cudaFreeHost(h_data);
	cudaFreeHost(dist);
	cudaFreeHost(thresh);
	cudaFreeHost(h_minDistance);
	cudaFreeHost(raw_segments);
	cudaFreeHost(car_segments);
	cudaFreeHost(h_relMeas);

	cudaFree(d_data);
	cudaFree(d_dist);
	cudaFree(d_thresh);
	cudaFree(d_minDistance);
	cudaFree(d_rawSegs);
	cudaFree(d_relMeas);
	cudaFree(d_carSegs);

	cudaStreamDestroy(stream1);
	cudaStreamDestroy(stream0);
}
__global__ void getRelevantMeas(cartesian_segment* carSegs, laserdata_cartesian* d_laser, unsigned long long* dist)
{
	int index = blockIdx.x*3;
	d_laser[index] = carSegs[blockIdx.x].measures[0];
	d_laser[index+2] = carSegs[blockIdx.x].measures[carSegs[blockIdx.x].numberOfMeasures-1];
	unsigned long long tmp;
	if(threadIdx.x < carSegs[blockIdx.x].numberOfMeasures)
	{
		float x = carSegs[blockIdx.x].measures[threadIdx.x].x;
		float y = carSegs[blockIdx.x].measures[threadIdx.x].y;
		tmp = sqrtf(x*x + y*y)*10000;
		atomicMin(&(dist[blockIdx.x]), tmp);
		__syncthreads();
		if(dist[blockIdx.x] == tmp)
		{
			d_laser[index+1] = carSegs[blockIdx.x].measures[threadIdx.x];
		}
	}
}

__device__ float computeEuclideanDistance(laserdata_raw p1, laserdata_raw p2)
{
	float square1 = p1.distance*p1.distance;
	float square2 = p2.distance*p2.distance;
	float deltaAlpha = p2.angle-p1.angle;
	deltaAlpha = (deltaAlpha * ((float)M_PI) / 180.0f);
	return sqrtf(square1+square2-2*p1.distance*p2.distance*cosf(deltaAlpha));
}
__device__ float computeThreshold(laserdata_raw p1, laserdata_raw p2)
{
	float C0 = 1.0f;
	float C1;
	float min_distance = p2.distance;
	float deltaAlpha;
	deltaAlpha = (0.25f * ((float)M_PI) / 180.0f);

	if(p1.distance < p2.distance)
	{
		min_distance = p1.distance;
	}

	C1 = sqrtf(2*(1-cosf(deltaAlpha)));

	return C0 + C1*min_distance;
}

__device__ void doCoordinateTransformDevice(raw_segment* rawSegs, cartesian_segment* carSegs, int segIndex, int laserIndex)
{
		carSegs[segIndex].numberOfMeasures = rawSegs[segIndex].numberOfMeasures;

		float angleInRadians;

		if(laserIndex < rawSegs[segIndex].numberOfMeasures)
		{
			angleInRadians = rawSegs[segIndex].measures[laserIndex].angle * ((float)M_PI) / 180.0f;
			carSegs[segIndex].measures[laserIndex].x = rawSegs[segIndex].measures[laserIndex].distance*cosf(angleInRadians);
			carSegs[segIndex].measures[laserIndex].y = rawSegs[segIndex].measures[laserIndex].distance*sinf(angleInRadians);
		}

}

__global__ void coordinateTransform(raw_segment* rawSegs, cartesian_segment* carSegs)
{
	doCoordinateTransformDevice(rawSegs, carSegs, blockIdx.x, threadIdx.x);
}
__global__ void processDist(laserdata_raw* data, float* distance)
{
	distance[threadIdx.x] = computeEuclideanDistance(data[threadIdx.x], data[threadIdx.x + 1]);
}
__global__ void processThresh(laserdata_raw* data, float* threshold)
{
	threshold[threadIdx.x] = computeThreshold(data[threadIdx.x], data[threadIdx.x + 1]);
}

/*
 * Reads the data out of the specified file and writes it to given array
 * @param
 * 	data: array for raw laserdata
 * @return
 * 	1 if an error occurs
 * 	0 if everything is ok
 */
int DataReader::getLaserData(laserdata_raw_array data, std::string number)
{
	std::ostringstream measurePath;
	measurePath << MEASUREPATH << number << ".txt";
#ifdef PRINT
	std::cout << measurePath.str() << std::endl;
#endif
	std::ifstream input(measurePath.str().c_str());
    std::string line;
    int counter = 0;
	std::string segment;
	/*
	 * Laserscanner range 145° with resolution of 0.25° -> 580 values + Zero value = 581
	 *-72,5° 0°  72,5°
	 * \    |    /
	 *  \   |   /
	 *   \  |  /
	 *    \ | /
	 *     \|/
	 *    Laser
	 */
	double angle = -72.5;

    //now read the data we are interested in
    counter = 0;
    int lineCounter = 0;
    while( std::getline( input, line ) && lineCounter < NUMBER_LASERRAYS ) {
    	//std::cout<<line<<'\n';
    	std::stringstream ss;

    	ss << line;

    	int datacnt = 1;
    	//extract relevant data from line
    	int valid = 0;
    	while(std::getline(ss, segment, ' '))
    	{
    		if(segment.size() > 0)
    		{
    			if(segment.at(0) < 48 || segment.at(0) > 57)
    			{
    				continue;
    			}
    			else
    			{
    				if(datacnt == 5)
    				{
    					valid = atoi(segment.c_str());
    					if(valid == 1)
    					{
    						data[counter].valid = valid;
    					}
    				}
    				else if(datacnt == 6 && valid == 1)
    				{
    					data[counter].distance = atof(segment.c_str());
    			    	break;
    				}
            		++datacnt;
    			}
    		}
    	}
    	if(valid)
    	{
    		data[counter].angle = angle;
	    	++counter;
    	}
    	angle += 0.25;
    	++lineCounter;
    }
#ifdef PRINT
    std::cout << counter << std::endl;
#endif
    return counter;
}
/**
 * Runs over all laser points and tries to group them to segments, regarding to their euclidean distance to their neighbor point
 * going from left to right
 */
int DataReader::processLaserData(std::string number, double currentSpeed, double currentYawRate, PointCellDevice* h_vehicles)
{
	this->currentSpeed = currentSpeed;
	this->currentYawRate = currentYawRate;

	//read new data from file
	int numElements = getLaserData(h_data, number);
	if(numElements < 3)
	{
		std::vector<PointCellDevice> vehicles;
		return 0;
	}
	cudaMemcpyAsync(d_data, h_data, numElements*sizeof(laserdata_raw), cudaMemcpyHostToDevice,stream1);
	cudaStreamSynchronize(stream1);
	processDist<<<1,numElements-1,0,stream1>>>(d_data, d_dist);
	processThresh<<<1, numElements-1,0, stream0>>>(d_data,d_thresh);
	for(int i=0; i<MAX_SEGMENTS; i++)
	{
		h_minDistance[i] = INT_MAX;
	}

	int segment_counter = 0;
	int data_counter = 0;

	//first point automatically is part of the first segment;
	raw_segments[MAX_SEGMENTS].numberOfMeasures = 1;
	raw_segments[MAX_SEGMENTS].measures[0] = h_data[0];

	cudaMemcpyAsync(dist, d_dist, (numElements-1)*sizeof(float), cudaMemcpyDeviceToHost, stream1);
	cudaMemcpyAsync(thresh, d_thresh, (numElements-1)*sizeof(float), cudaMemcpyDeviceToHost, stream0);
	cudaStreamSynchronize(stream1);
	cudaStreamSynchronize(stream0);
	cudaMemcpyAsync(d_minDistance, h_minDistance, MAX_SEGMENTS*sizeof(unsigned long long), cudaMemcpyHostToDevice,stream0);
	//iterate over all measures
	for(int i=1; i<numElements; i++)
	{
		if(dist[i-1] <= thresh[i-1])
		{
			//add current point in existing segment
			raw_segments[MAX_SEGMENTS].numberOfMeasures++;
			raw_segments[MAX_SEGMENTS].measures[++data_counter] = h_data[i];
		}
		else
		{
			//point belongs to other object -> store current Segment and reset tmp-segment object
			//only keep segments with at least 3 points
			if(raw_segments[MAX_SEGMENTS].numberOfMeasures >= 3)
			{
				raw_segments[segment_counter].numberOfMeasures = raw_segments[MAX_SEGMENTS].numberOfMeasures;
				for(int j=0; j<raw_segments[MAX_SEGMENTS].numberOfMeasures; j++)
				{
					raw_segments[segment_counter].measures[j] = raw_segments[MAX_SEGMENTS].measures[j];
				}
				segment_counter++;
			}
			raw_segments[MAX_SEGMENTS].numberOfMeasures = 1;
			raw_segments[MAX_SEGMENTS].measures[0] = h_data[i];
			data_counter = 0;
		}
	}

	if(raw_segments[MAX_SEGMENTS].numberOfMeasures >= 3)
	{
		raw_segments[segment_counter].numberOfMeasures = raw_segments[MAX_SEGMENTS].numberOfMeasures;
		for(int j=0; j<raw_segments[MAX_SEGMENTS].numberOfMeasures; j++)
		{
			raw_segments[segment_counter].measures[j] = raw_segments[MAX_SEGMENTS].measures[j];
		}
		++segment_counter;
	}
#ifdef PRINT
	printf("Extracted %d Objects from Laserdata\n", segment_counter);
#endif
	cudaMemcpyAsync(d_rawSegs, raw_segments, segment_counter*sizeof(raw_segment), cudaMemcpyHostToDevice,stream1);
	coordinateTransform<<<segment_counter,NUMBER_LASERRAYS,0,stream1>>>(d_rawSegs, d_carSegs);
	cudaStreamSynchronize(stream1);
	cudaMemcpyAsync(car_segments, d_carSegs, segment_counter*sizeof(cartesian_segment), cudaMemcpyDeviceToHost,stream1);
	getRelevantMeas<<<segment_counter, NUMBER_LASERRAYS,0,stream0>>>(d_carSegs,d_relMeas,d_minDistance);
#ifdef VISUALIZE
	visualizer.visualizeSegmentsAsPointCloud(car_segments, number, segment_counter);
#endif
	cudaMemcpyAsync(h_relMeas, d_relMeas, segment_counter*3*sizeof(laserdata_cartesian), cudaMemcpyDeviceToHost,stream0);
	cudaStreamSynchronize(stream0);
	cudaStreamSynchronize(stream1);
	int vehicles = computeVehicleState(car_segments, segment_counter, number, h_vehicles);
	return vehicles;
}

int DataReader::computeVehicleState(cartesian_segment* segments, int segmentCounter, std::string number, PointCellDevice* h_vehicles)
{
	std::vector<PointCellDevice> vehicles;
	int counter = 0;
	laserdata_cartesian* relevantPoints;
	std::vector<std::vector<laserdata_cartesian> > toPlot;

	for(uint i=0; i<segmentCounter; i++)
	{
		relevantPoints = &h_relMeas[i*3];

		//we have three different points, compute bounds

		int left = 0;
		//right point - left point
		double width = fabs(relevantPoints[2].y - relevantPoints[0].y);
		double length = fabs(relevantPoints[2].x - relevantPoints[0].x);
		double nearestLengthLeft = fabs(relevantPoints[1].x - relevantPoints[0].x);
		double nearestLengthRight = fabs(relevantPoints[1].x - relevantPoints[2].x);
		double nearestWidthLeft = fabs(relevantPoints[1].y - relevantPoints[0].y);
		double nearestWidthRight = fabs(relevantPoints[1].y - relevantPoints[2].y);
		//compute orientation of object regarding to the driving direction of our own car
		//own direction vector(x,y): (1,0)

		if(length > 2)
		{
			length = fabs(relevantPoints[1].x - relevantPoints[0].x);
			width = fabs(relevantPoints[1].y - relevantPoints[0].y);
		}

		double theta = acos(length/(1*sqrt(width*width + length*length))) * 180.0 / M_PI;

		double thetaLeft = acos(nearestLengthLeft/(1*sqrt(nearestWidthLeft*nearestWidthLeft + nearestLengthLeft*nearestLengthLeft))) * 180.0 / M_PI;

		double thetaRight = acos(nearestLengthRight/(1*sqrt(nearestWidthRight*nearestWidthRight + nearestLengthRight*nearestLengthRight))) * 180.0 / M_PI;

		//objects should not be classified as vehicle if their orientation is bigger than 45°
		//real vehicles should never be rotated over that value

		//the detected car probably is defined with the points that form the biggest angle and are wider than 1m
		int points = 0;
		if(thetaLeft + 5 > theta && nearestWidthLeft > 1)
		{
			theta = thetaLeft;
			length = nearestLengthLeft;
			width = nearestWidthLeft;
			points = 1;
		}
		if(thetaRight + 5 > theta && nearestWidthRight > 1)
		{
			theta = thetaRight;
			length = nearestLengthRight;
			width = nearestWidthRight;
			left = 1;
			points = 2;
		}
		if(theta > 60 && width > 1)
		{
			double y = relevantPoints[left].y + width/2;
			h_vehicles[counter].initializeMemory();
			h_vehicles[counter].setX(relevantPoints[left].x + length/2);//x
			h_vehicles[counter].setY(relevantPoints[left].y + width/2); // y
			//now compute theta regarding to movement direction
			switch(points)
			{
			case 0:
				width = (relevantPoints[2].y - relevantPoints[0].y);
				length = (relevantPoints[2].x - relevantPoints[0].x);
				break;
			case 1:
				length = (relevantPoints[1].x - relevantPoints[0].x);
				width = (relevantPoints[1].y - relevantPoints[0].y);
				break;
			case 2:
				length = (relevantPoints[1].x - relevantPoints[2].x);
				width = (relevantPoints[1].y - relevantPoints[2].y);
				break;
			}
			theta = atan(width/length);
			h_vehicles[counter].setTheta(theta*M_PI / 180.0); //theta
			//due to prior knowledge on highways, velocitys for diffrent lanes are estimated as below
			if(y < -4.5)
			{
				h_vehicles[counter].setVelocity(currentSpeed + 11.11); //velocity + 40kmh
			}
			else if(y < -1.5)
			{
				h_vehicles[counter].setVelocity(currentSpeed + 5.55); //velocity + 20kmh
			}
			else if(y > 4.5)
			{
				h_vehicles[counter].setVelocity(currentSpeed - 11.11); //velocity - 40kmh
			}
			else if(y > 1.5)
			{
				h_vehicles[counter].setVelocity(currentSpeed - 5.55); //velocity - 20kmh
			}
			else
			{
				h_vehicles[counter].setVelocity(currentSpeed); //velocity
			}
			h_vehicles[counter].setPhi(currentYawRate); //yaw rate
			h_vehicles[counter].subInvtl = 0.5;
			++counter;
			std::vector<laserdata_cartesian> tmp;
			tmp.push_back(relevantPoints[0]);
			tmp.push_back(relevantPoints[1]);
			tmp.push_back(relevantPoints[2]);
			toPlot.push_back(tmp);
		}
	}
#ifdef PRINT
	std::cout<<"Extracted " << counter << " Vehicles from Data" << std::endl;
#endif
#ifdef VISUALIZE
	visualizer.visualizeVehiclesAsRectangle(toPlot, number);
#endif
	return counter;
}
