/*
 * DataReader.cpp
 *
 *  Created on: 06.06.2016
 *      Author: Sebastian Reinhart
 */

#include "DataReader.cuh"


DataReader::DataReader() {
	// TODO Auto-generated constructor stub
	cudaError_t error;
	cudaStreamCreate(&stream1);
	cudaStreamCreate(&stream0);
	error = cudaHostAlloc((void**) &h_data, NUMBER_LASERRAYS*sizeof(laserdata_raw), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_data_ptr, h_data, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &dist, NUMBER_LASERRAYS*sizeof(double), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_dist_ptr, dist, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &h_relMeas, MAX_SEGMENTS*3*sizeof(laserdata_cartesian), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_relMeas_ptr, h_relMeas, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &thresh, NUMBER_LASERRAYS*sizeof(double), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_thresh_ptr, thresh, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &raw_segments, (MAX_SEGMENTS+1)*sizeof(raw_segment), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_rawSegs_ptr, raw_segments, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &car_segments, (MAX_SEGMENTS+1)*sizeof(cartesian_segment), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_carSegs_ptr, car_segments, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error =cudaHostAlloc((void **) &h_minDistance, MAX_SEGMENTS*sizeof(unsigned long long), cudaHostAllocMapped);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_minDistance_ptr, h_minDistance, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}
}

DataReader::~DataReader() {
	// TODO Free device memory
	cudaFreeHost(h_data);
	cudaFreeHost(dist);
	cudaFreeHost(thresh);
	cudaFreeHost(h_minDistance);
	cudaFreeHost(raw_segments);
	cudaFreeHost(car_segments);
	cudaStreamDestroy(stream1);
	cudaStreamDestroy(stream0);
}
__global__ void getRelevantMeas(cartesian_segment* carSegs, laserdata_cartesian* d_laser, unsigned long long* dist)
{
//	printf("Block %d entered Min\n", blockIdx.x);
	int index = blockIdx.x*3;
	d_laser[index] = carSegs[blockIdx.x].measures[0];
	d_laser[index+2] = carSegs[blockIdx.x].measures[carSegs[blockIdx.x].numberOfMeasures-1];
	unsigned long long tmp;
	if(threadIdx.x < carSegs[blockIdx.x].numberOfMeasures)
	{
		double x = carSegs[blockIdx.x].measures[threadIdx.x].x;
		double y = carSegs[blockIdx.x].measures[threadIdx.x].y;
		tmp = sqrt(x*x + y*y)*10000;
		atomicMin(&(dist[blockIdx.x]), tmp);
		__syncthreads();
		if(dist[blockIdx.x] == tmp)
		{
			d_laser[index+1] = carSegs[blockIdx.x].measures[threadIdx.x];
		}
	}
}

__global__ void extractVehicles(laserdata_cartesian* d_laser, PointCellDevice* d_vehicles, unsigned int* index, double currentSpeed, double currentYawRate)
{

//	currentSegment = segments.at(i);
	//relevantPoints = getRelevantMeasuresFromSegment(currentSegment);


	//we have three different points, compute bounds

	int left = 0;
	//right point - left point
	double width = fabs(d_laser[blockIdx.x+2].y - d_laser[blockIdx.x].y);
	double length = fabs(d_laser[blockIdx.x+2].x - d_laser[blockIdx.x].x);
	double nearestLengthLeft = fabs(d_laser[blockIdx.x+1].x - d_laser[blockIdx.x].x);
	double nearestLengthRight = fabs(d_laser[blockIdx.x+1].x - d_laser[blockIdx.x+2].x);
	double nearestWidthLeft = fabs(d_laser[blockIdx.x+1].y - d_laser[blockIdx.x].y);
	double nearestWidthRight = fabs(d_laser[blockIdx.x+1].y - d_laser[blockIdx.x+2].y);
	//compute orientation of object regarding to the driving direction of our own car
	//own direction vector(x,y): (1,0)
/*	std::cout << i << std::endl;
	std::cout << "left x = " << relevantPoints[0].x << " y = " << relevantPoints[0].y << std::endl;
	std::cout << "nearest x = " << relevantPoints[1].x << " y = " << relevantPoints[1].y << std::endl;
	std::cout << "right x = " << relevantPoints[2].x << " y = " << relevantPoints[2].y << std::endl;*/

	if(length > 2)
	{
		length = fabs(d_laser[blockIdx.x+1].x - d_laser[blockIdx.x].x);
		width = fabs(d_laser[blockIdx.x+1].y - d_laser[blockIdx.x].y);
	}

	double theta = acos(length/(1*sqrt(width*width + length*length))) * 180.0 / M_PI;

	double thetaLeft = acos(nearestLengthLeft/(1*sqrt(nearestWidthLeft*nearestWidthLeft + nearestLengthLeft*nearestLengthLeft))) * 180.0 / M_PI;

	double thetaRight = acos(nearestLengthRight/(1*sqrt(nearestWidthRight*nearestWidthRight + nearestLengthRight*nearestLengthRight))) * 180.0 / M_PI;

	//objects should not be classified as vehicle if their orientation is bigger than 45°
	//real vehicles should never be rotated over that value

	/*	std::cout << "Theta: " << theta << std::endl;
		std::cout << "ThetaLeft: " << thetaLeft << std::endl;
		std::cout << "ThetaRight: " << thetaRight << std::endl;
		std::cout << "Length: " << length << std::endl;
		std::cout << "Width: " << width << std::endl;*/

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
		int localIndex = atomicInc(index, 100000);
		//PointCellDevice vehicle;
		//vehicle.width = width;
		double y = d_laser[blockIdx.x+left].y + width/2;
		d_vehicles[localIndex].setX(d_laser[blockIdx.x+left].x + length/2);//x
		d_vehicles[localIndex].setY(d_laser[blockIdx.x+left].y + width/2); // y
		//now compute theta regarding to movement direction
		switch(points)
		{
		case 0:
			width = (d_laser[blockIdx.x+2].y - d_laser[blockIdx.x].y);
			length = (d_laser[blockIdx.x+2].x - d_laser[blockIdx.x].x);
			break;
		case 1:
			length = (d_laser[blockIdx.x+1].x - d_laser[blockIdx.x].x);
			width = (d_laser[blockIdx.x+1].y - d_laser[blockIdx.x].y);
			break;
		case 2:
			length = (d_laser[blockIdx.x+1].x - d_laser[blockIdx.x+2].x);
			width = (d_laser[blockIdx.x+1].y - d_laser[blockIdx.x+2].y);
			break;
		}
		theta = atan(width/length);
		d_vehicles[localIndex].setTheta(theta*M_PI / 180.0); //theta
		//due to prior knowledge on highways, velocitys for diffrent lanes are estimated as below
		if(y < -4.5)
		{
			d_vehicles[localIndex].setVelocity(currentSpeed + 11.11); //velocity + 40kmh
		}
		else if(y < -1.5)
		{
			d_vehicles[localIndex].setVelocity(currentSpeed + 5.55); //velocity + 20kmh
		}
		else if(y > 4.5)
		{
			d_vehicles[localIndex].setVelocity(currentSpeed - 11.11); //velocity - 40kmh
		}
		else if(y > 1.5)
		{
			d_vehicles[localIndex].setVelocity(currentSpeed - 5.55); //velocity - 20kmh
		}
		else
		{
			d_vehicles[localIndex].setVelocity(currentSpeed); //velocity
		}
		d_vehicles[localIndex].setPhi(currentYawRate); //yaw rate
		d_vehicles[localIndex].subInvtl = 0.5;
	}
}


//Definitely to slow run sequential on GPU
__device__ int segmentData(laserdata_raw* data, raw_segment* rawSegs, double* dist, double* threshold, int numElements)
{
	int segment_counter = 0;
	int data_counter = 0;
	//oldMeasure = data[0];
	//first point automatically is part of the first segment;
	rawSegs[MAX_SEGMENTS].numberOfMeasures = 1;
	rawSegs[MAX_SEGMENTS].measures[0] = data[0];
//	cudaThreadSynchronize();
//	cudaMemcpy(dist, d_dist, size_double, cudaMemcpyDeviceToHost);
//	cudaMemcpy(thresh, d_thresh, size_double, cudaMemcpyDeviceToHost);
//	cudaFree(d_data);
//	cudaFree(d_dist);
//	cudaFree(d_thresh);
	//iterate over all measures
	for(int i=1; i<numElements; i++)
	{
		//currentMeasure = data[i];
	//	std::cout << "Distance " << dist[i-1] << " Threshold " << thresh[i-1] << std::endl;
		if(dist[i-1] <= threshold[i-1])
		{
			//add current point in existing segment
			rawSegs[MAX_SEGMENTS].numberOfMeasures++;
			rawSegs[MAX_SEGMENTS].measures[++data_counter] = data[i];
		}
		else
		{
			//point belongs to other object -> store current Segment and reset tmp-segment object
			//only keep segments with at least 3 points
			if(rawSegs[MAX_SEGMENTS].numberOfMeasures >= 3)
			{
				rawSegs[segment_counter].numberOfMeasures = rawSegs[MAX_SEGMENTS].numberOfMeasures;
				for(int j=0; j<rawSegs[MAX_SEGMENTS].numberOfMeasures; j++)
				{
					rawSegs[segment_counter].measures[j] = rawSegs[MAX_SEGMENTS].measures[j];
				}
				segment_counter++;
			}
			rawSegs[MAX_SEGMENTS].numberOfMeasures = 1;
			//currentSegment.measures.clear();
			rawSegs[MAX_SEGMENTS].measures[0] = data[i];
			data_counter = 0;
//			printf("Finished segment with %d points!")
		}
	//	oldMeasure = currentMeasure;
	}

	if(rawSegs[MAX_SEGMENTS].numberOfMeasures >= 3)
	{
		rawSegs[segment_counter].numberOfMeasures = rawSegs[MAX_SEGMENTS].numberOfMeasures;
		for(int j=0; j<rawSegs[MAX_SEGMENTS].numberOfMeasures; j++)
		{
			rawSegs[segment_counter].measures[j] = rawSegs[MAX_SEGMENTS].measures[j];
		}
		++segment_counter;
	}

//	printf("Extracted %d Objects from Laserdata\n", segment_counter);

	return segment_counter;
}

__global__ void segmentation(laserdata_raw* data, raw_segment* rawSegs, double* dist, double* threshold, int numElements, int* d_numSegments)
{
	*d_numSegments = segmentData(data, rawSegs, dist, threshold, numElements);
}
__device__ double computeEuclideanDistance(laserdata_raw p1, laserdata_raw p2)
{
	double square1 = p1.distance*p1.distance;
	double square2 = p2.distance*p2.distance;
	double deltaAlpha = p2.angle-p1.angle;
	deltaAlpha = (deltaAlpha * M_PI / 180.0);
	return sqrt(square1+square2-2*p1.distance*p2.distance*cos(deltaAlpha));
}
__device__ double computeThreshold(laserdata_raw p1, laserdata_raw p2)
{
	//https://www.researchgate.net/publication/243773062_Model_Based_Object_Classification_and_Tracking_in_Traffic_Scenes_from_Range_Images
	double C0 = 1.0;
	double C1;
	double min_distance = p2.distance;
	double deltaAlpha;
	deltaAlpha = (0.25 * M_PI / 180.0);

	if(p1.distance < p2.distance)
	{
		min_distance = p1.distance;
	}

	C1 = sqrt(2*(1-cos(deltaAlpha)));

	return C0 + C1*min_distance;
}

__device__ void doCoordinateTransformDevice(raw_segment* rawSegs, cartesian_segment* carSegs, int segIndex, int laserIndex)
{
		carSegs[segIndex].numberOfMeasures = rawSegs[segIndex].numberOfMeasures;

		double angleInRadians;

		if(laserIndex < rawSegs[segIndex].numberOfMeasures)
		{
			angleInRadians = rawSegs[segIndex].measures[laserIndex].angle * M_PI / 180.0;
			carSegs[segIndex].measures[laserIndex].x = rawSegs[segIndex].measures[laserIndex].distance*cos(angleInRadians);
			carSegs[segIndex].measures[laserIndex].y = rawSegs[segIndex].measures[laserIndex].distance*sin(angleInRadians);
		}

}

__global__ void coordinateTransform(raw_segment* rawSegs, cartesian_segment* carSegs)
{
	doCoordinateTransformDevice(rawSegs, carSegs, blockIdx.x, threadIdx.x);
}
__global__ void processDist(laserdata_raw* data, double* distance)
{
	distance[threadIdx.x] = computeEuclideanDistance(data[threadIdx.x], data[threadIdx.x + 1]);
}
__global__ void processThresh(laserdata_raw* data, double* threshold)
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

    //Skip first 581 lines, just read out the second level of datas
  /*  while(counter < NUMBER_LASERRAYS && std::getline( input, line )) {
    	++counter;
  //  	std::cout<< counter <<'\n';
    }*/

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
    		//		std::cout<<segment << ' ';
            		++datacnt;
    			}
    		}
    	}
    	if(valid)
    	{
    		data[counter].angle = angle;
    //		std::cout<< "Angle: " << data[counter].angle << " Distance: " << data[counter].distance << " Valid: " << data[counter].valid << '\n';
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
	processDist<<<1,numElements-1,0,stream1>>>(d_data_ptr, d_dist_ptr);
	processThresh<<<1, numElements-1,0, stream0>>>(d_data_ptr,d_thresh_ptr);
	for(int i=0; i<MAX_SEGMENTS; i++)
	{
		h_minDistance[i] = INT_MAX;
	}

	int segment_counter = 0;
	int data_counter = 0;

	//first point automatically is part of the first segment;
	raw_segments[MAX_SEGMENTS].numberOfMeasures = 1;
	raw_segments[MAX_SEGMENTS].measures[0] = h_data[0];
	cudaStreamSynchronize(stream1);
	cudaStreamSynchronize(stream0);
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
	coordinateTransform<<<segment_counter,NUMBER_LASERRAYS,0,stream1>>>(d_rawSegs_ptr, d_carSegs_ptr);
	cudaStreamSynchronize(stream1);
	getRelevantMeas<<<segment_counter, NUMBER_LASERRAYS,0,stream0>>>(d_carSegs_ptr,d_relMeas_ptr,d_minDistance_ptr);
#ifdef VISUALIZE
	visualizer.visualizeSegmentsAsPointCloud(car_segments, number, segment_counter);
#endif
	cudaStreamSynchronize(stream0);
	int vehicles = computeVehicleState(car_segments, segment_counter, number, h_vehicles);
	return vehicles;
}

/*
 * ANMERKUNG: KÖNNTE FÜR GESAMTES ARRAY PARALLEL BERECHNET WERDEN!!
 *
 */
double DataReader::computeEuclideanDistance(laserdata_raw p1, laserdata_raw p2)
{
	double square1 = p1.distance*p1.distance;
	double square2 = p2.distance*p2.distance;
	double deltaAlpha = p2.angle-p1.angle;
	deltaAlpha = (deltaAlpha * M_PI / 180.0);
	return sqrt(square1+square2-2*p1.distance*p2.distance*cos(deltaAlpha));
}

double DataReader::computeThreshold(laserdata_raw p1, laserdata_raw p2)
{
	//https://www.researchgate.net/publication/243773062_Model_Based_Object_Classification_and_Tracking_in_Traffic_Scenes_from_Range_Images
	double C0 = 1.0;
	double C1;
	double min_distance = p2.distance;

	double deltaAlpha;
	deltaAlpha = (0.25 * M_PI / 180.0);

	if(p1.distance < p2.distance)
	{
		min_distance = p1.distance;
	}

	C1 = sqrt(2*(1-cos(deltaAlpha)));

	return C0 + C1*min_distance;

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
		//	PointCellDevice vehicle;
			//vehicle.width = width;
			double y = relevantPoints[left].y + width/2;
			h_vehicles[counter].initializeMemory();
		//	vehicle.setX(relevantPoints[left].x + length/2);//x
		//	vehicle.setY(relevantPoints[left].y + width/2); // y
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
			//vehicle.setTheta(theta*M_PI / 180.0); //theta
			h_vehicles[counter].setTheta(theta*M_PI / 180.0); //theta
			//due to prior knowledge on highways, velocitys for diffrent lanes are estimated as below
			if(y < -4.5)
			{
				//vehicle.setVelocity(currentSpeed + 11.11); //velocity + 40kmh
				h_vehicles[counter].setVelocity(currentSpeed + 11.11); //velocity + 40kmh
			}
			else if(y < -1.5)
			{
				//vehicle.setVelocity(currentSpeed + 5.55); //velocity + 20kmh
				h_vehicles[counter].setVelocity(currentSpeed + 5.55); //velocity + 20kmh
			}
			else if(y > 4.5)
			{
				//vehicle.setVelocity(currentSpeed - 11.11); //velocity - 40kmh
				h_vehicles[counter].setVelocity(currentSpeed - 11.11); //velocity - 40kmh
			}
			else if(y > 1.5)
			{
				//vehicle.setVelocity(currentSpeed - 5.55); //velocity - 20kmh
				h_vehicles[counter].setVelocity(currentSpeed - 5.55); //velocity - 20kmh
			}
			else
			{
				//vehicle.setVelocity(currentSpeed); //velocity
				h_vehicles[counter].setVelocity(currentSpeed); //velocity
			}
		//	vehicle.setPhi(currentYawRate); //yaw rate
		//	vehicle.subInvtl = 0.5;
			h_vehicles[counter].setPhi(currentYawRate); //yaw rate
			h_vehicles[counter].subInvtl = 0.5;
			//vehicles.push_back(vehicle);
			++counter;
			std::vector<laserdata_cartesian> tmp;
			tmp.push_back(relevantPoints[0]);
			tmp.push_back(relevantPoints[1]);
			tmp.push_back(relevantPoints[2]);
			toPlot.push_back(tmp);
		}
	}
#ifdef PRINT
	std::cout<<"Extracted " << toPlot.size() << " Vehicles from Data" << std::endl;
#endif
#ifdef VISUALIZE
	visualizer.visualizeVehiclesAsRectangle(toPlot, number);
#endif
	return counter;
}

/**
 * Returns the a vector containing the left border, the nearest point and the right border of a given segment
 */
std::vector<laserdata_cartesian> DataReader::getRelevantMeasuresFromSegment(cartesian_segment segment)
{
	std::vector<laserdata_cartesian> relevantMeasures;
	double leastMeasure = INT_MAX;
	laserdata_cartesian leastLaser;
	//left border is always the first measure in a segment
	relevantMeasures.push_back(segment.measures[0]);

	//Search for the measure with the least distance to ourself
	double tmp;
	laserdata_cartesian tmpLaser;
	for(int i=0; i<segment.numberOfMeasures; i++)
	{
		tmpLaser = segment.measures[i];
		tmp = sqrt(tmpLaser.x * tmpLaser.x + tmpLaser.y * tmpLaser.y);
		if(tmp < leastMeasure)
		{
			leastLaser = tmpLaser;
			leastMeasure = tmp;
		}
	}
	relevantMeasures.push_back(leastLaser);
	//right border is always the last measure in a segment
	relevantMeasures.push_back(segment.measures[segment.numberOfMeasures-1]);

	return relevantMeasures;
}


