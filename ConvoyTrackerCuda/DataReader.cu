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
	segments = std::vector<raw_segment>(MAX_SEGMENTS+1);
	for(int i=0; i<MAX_SEGMENTS+1; i++)
	{
//		segments.at(i).measures = (laserdata_raw*) malloc(NUMBER_LASERRAYS*sizeof(laserdata_raw));
		error = cudaHostAlloc((void**) &segments.at(i).measures, NUMBER_LASERRAYS*sizeof(laserdata_raw), cudaHostAllocDefault);
		if (error != cudaSuccess) {
			printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
					cudaGetErrorString(error), error, __LINE__);
		}
	}


	error = cudaHostAlloc((void**) &h_data, NUMBER_LASERRAYS*sizeof(laserdata_raw), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &dist, NUMBER_LASERRAYS*sizeof(double), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostAlloc((void**) &thresh, NUMBER_LASERRAYS*sizeof(double), cudaHostAllocDefault);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void **) &d_rawSegs, (MAX_SEGMENTS+1)*sizeof(raw_segment));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}
	error = cudaMalloc((void **) &d_carSegs, (MAX_SEGMENTS+1)*sizeof(cartesian_segment));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}
	error =cudaMalloc((void **) &d_carLaser, MAX_SEGMENTS*3*sizeof(laserdata_cartesian));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaMalloc((void**) &d_vehicles, (MAX_SEGMENTS+1)*sizeof(PointCellDevice));
	if (error != cudaSuccess) {
			printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
			cudaGetErrorString(error), error, __LINE__);
	}

	error =cudaMalloc((void **) &d_minDistance, MAX_SEGMENTS*sizeof(unsigned long long));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error =cudaMalloc((void **) &d_index, sizeof(unsigned int));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error =cudaMalloc((void **) &d_numSegments, sizeof(int));
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	for(int i=0; i<MAX_SEGMENTS+1; i++)
	{
		laserdata_raw* tmp;
		error = cudaMalloc((void **) &tmp, NUMBER_LASERRAYS*sizeof(laserdata_raw));
		if (error != cudaSuccess) {
			printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
					cudaGetErrorString(error), error, __LINE__);
		}
		d_rawMeasure.push_back(tmp);

		laserdata_cartesian* tmp2;
		error = cudaMalloc((void **) &tmp2, NUMBER_LASERRAYS*sizeof(laserdata_cartesian));
		if (error != cudaSuccess) {
			printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
					cudaGetErrorString(error), error, __LINE__);
		}
		d_carMeasure.push_back(tmp2);

		double* tmp3;
		error = cudaMalloc((void **) &tmp3, 260*sizeof(double));
		if (error != cudaSuccess) {
			printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
					cudaGetErrorString(error), error, __LINE__);
		}
		d_vehicleData.push_back(tmp3);
	}

	//copy pointers to allocated device storage
	for(int i=0; i<(MAX_SEGMENTS+1); i++)
	{
		error = cudaMemcpy(&((d_rawSegs+i)->measures), &(d_rawMeasure.data()[i]), sizeof(laserdata_raw*), cudaMemcpyHostToDevice);
		if (error != cudaSuccess) {
			printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
					cudaGetErrorString(error), error, __LINE__);
		}
		error = cudaMemcpy(&((d_carSegs+i)->measures), &(d_carMeasure.data()[i]), sizeof(laserdata_cartesian*), cudaMemcpyHostToDevice);
		if (error != cudaSuccess) {
			printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
					cudaGetErrorString(error), error, __LINE__);
		}
		error = cudaMemcpy(&((d_vehicles+i)->data), &(d_vehicleData.data()[i]), sizeof(double*), cudaMemcpyHostToDevice);
		if (error != cudaSuccess) {
			printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
					cudaGetErrorString(error), error, __LINE__);
		}
	}



		size_t size_struct = NUMBER_LASERRAYS*sizeof(laserdata_raw);
		size_t size_double = (NUMBER_LASERRAYS - 1)*sizeof(double);
		error = cudaMalloc((void **) &d_data, size_struct);
		if (error != cudaSuccess) {
			printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
					cudaGetErrorString(error), error, __LINE__);
		}
		error = cudaMalloc((void **) &d_dist, size_double);
		if (error != cudaSuccess) {
			printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
					cudaGetErrorString(error), error, __LINE__);
		}
		error = cudaMalloc((void **) &d_thresh, size_double);
		if (error != cudaSuccess) {
			printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
					cudaGetErrorString(error), error, __LINE__);
		}
}

DataReader::~DataReader() {
	// TODO Free device memory
	for(int i=0; i<MAX_SEGMENTS+1; i++)
	{
		cudaFreeHost(segments.at(i).measures);
		cudaFree(d_carMeasure.at(i));
		cudaFree(d_rawMeasure.at(i));
		cudaFree(d_vehicleData.at(i));
	}
	cudaFreeHost(h_data);
	cudaFreeHost(dist);
	cudaFreeHost(thresh);
	cudaFree(d_data);
	cudaFree(d_carSegs);
	cudaFree(d_rawSegs);
	cudaFree(d_dist);
	cudaFree(d_thresh);
	cudaFree(d_index);
	cudaFree(d_minDistance);
	cudaFree(d_numSegments);
	cudaFree(d_vehicles);
	cudaStreamDestroy(stream1);
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
			printf("Block %d Completed Min\n", blockIdx.x);
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

//	C2= this->computeEuclideanDistance(p1,p2)/p1.distance;
	C1 = sqrt(2*(1-cos(deltaAlpha)));

	return C0 + C1*min_distance;
}

__device__ void doCoordinateTransformDevice(raw_segment* rawSegs, cartesian_segment* carSegs, int segIndex, int laserIndex)
{

	//	cartesian_segment curSeg;
	//	curSeg.numberOfMeasures = seg.numberOfMeasures;
		carSegs[segIndex].numberOfMeasures = rawSegs[segIndex].numberOfMeasures;

		double angleInRadians;

	//	for(int j=0; j<rawSegs[index].numberOfMeasures; j++)
	//	{
		//	laserdata_raw raw = seg.measures.at(j);
		//	laserdata_cartesian currentLaser;
			if(laserIndex < rawSegs[segIndex].numberOfMeasures)
			{
				angleInRadians = rawSegs[segIndex].measures[laserIndex].angle * M_PI / 180.0;

				carSegs[segIndex].measures[laserIndex].x = rawSegs[segIndex].measures[laserIndex].distance*cos(angleInRadians);
				carSegs[segIndex].measures[laserIndex].y = rawSegs[segIndex].measures[laserIndex].distance*sin(angleInRadians);
			}


	//		curSeg.measures.push_back(currentLaser);
	//	}
}

__global__ void coordinateTransform(raw_segment* rawSegs, cartesian_segment* carSegs)
{
	doCoordinateTransformDevice(rawSegs, carSegs, blockIdx.x, threadIdx.x);
}
__global__ void processData(laserdata_raw* data, raw_segment* rawSegs, cartesian_segment* carSegs, double* distance, double* threshold, int numElements, int* d_numSegments)
{
//	printf("Thread %d entered kernel\n", threadIdx.x);
	distance[threadIdx.x] = computeEuclideanDistance(data[threadIdx.x], data[threadIdx.x + 1]);
	threshold[threadIdx.x] = computeThreshold(data[threadIdx.x], data[threadIdx.x + 1]);
//	printf("Thread %d finished Dist/Thresh computation\n", threadIdx.x);
/*	int l_numSegments;
	if(threadIdx.x == 0)
	{
	//	printf("Thread %d before Segmentation\n", threadIdx.x);
		l_numSegments = segmentData(data,rawSegs,distance,threshold,numElements);
	//	printf("Thread %d after Segmentation\n", threadIdx.x);
		*d_numSegments = l_numSegments;
	}*/
/*	__syncthreads();
//	printf("Num Segs %d\n",l_numSegments);
	if(threadIdx.x < l_numSegments)
	{
		doCoordinateTransformDevice(rawSegs, carSegs, threadIdx.x);
	}*/
//	printf("Thread %d left kernel\n", threadIdx.x);

}

__global__ void testMemory(raw_segment* rawSegs)
{
	printf("Entered kernel\n");
	rawSegs[0].numberOfMeasures = 1;
	rawSegs[0].measures[0].valid = 1;
	rawSegs[0].measures[0].angle = 2;
	rawSegs[0].measures[0].distance = 3;
	printf("Left kernel\n");
}

/*__global__ void transformSegments(raw_segment* segments, cartesian_segment* transformed_segments, DataVisualizer* visualizer, std::string* number)
{
	transformed_segments[threadIdx.x] = doCoordinateTransform(segments[threadIdx.x]);
	visualizer->visualizeSegmentsAsPointCloud(transformed_segments[threadIdx.x],*number);
}*/
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
	std::cout << measurePath.str() << std::endl;
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
    std::cout << counter << std::endl;
    return counter;
}
/**
 * Runs over all laser points and tries to group them to segments, regarding to their euclidean distance to their neighbor point
 * going from left to right
 */
std::vector<PointCellDevice> DataReader::processLaserData(std::string number, double currentSpeed, double currentYawRate)
{
	this->currentSpeed = currentSpeed;
	this->currentYawRate = currentYawRate;

	//read new data from file
	cudaEvent_t startEvent, stopEvent;
	cudaEventCreate(&startEvent);
	cudaEventCreate(&stopEvent);
	cudaEventRecord(startEvent, 0);
	int numElements = getLaserData(h_data, number);
	cudaEventRecord(stopEvent, 0);
	cudaEventSynchronize(stopEvent);
	float time;
	cudaEventElapsedTime(&time, startEvent, stopEvent);
	std::cout << "Read data Time: " << time << std::endl;

	cudaError_t error;

	size_t size_struct = numElements*sizeof(laserdata_raw);

	error = cudaMemcpyAsync(d_data, h_data, size_struct, cudaMemcpyHostToDevice,stream1);
/*	error = cudaHostGetDevicePointer(&d_data_ptr, h_data, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_dist_ptr, dist, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	error = cudaHostGetDevicePointer(&d_thresh_ptr, thresh, 0);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}*/

	processData<<<1,numElements-1,0,stream1>>>(d_data, d_rawSegs, d_carSegs, d_dist, d_thresh, numElements, d_numSegments);

	error = cudaMemcpyAsync(dist, d_dist, (numElements-1)*sizeof(double), cudaMemcpyDeviceToHost,stream1);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}
	error = cudaMemcpyAsync(thresh, d_thresh, (numElements-1)*sizeof(double), cudaMemcpyDeviceToHost,stream1);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}
/*	error = cudaMemcpy(&h_numSegments, d_numSegments, sizeof(int), cudaMemcpyDeviceToHost);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}*/

	cudaStreamSynchronize(stream1);

	cudaEventRecord(startEvent, 0);
	int segment_counter = 0;
	int data_counter = 0;

	//first point automatically is part of the first segment;
	segments[MAX_SEGMENTS].numberOfMeasures = 1;
	segments[MAX_SEGMENTS].measures[0] = h_data[0];
	//iterate over all measures
	for(int i=1; i<numElements; i++)
	{
		if(dist[i-1] <= thresh[i-1])
		{
			//add current point in existing segment
			segments[MAX_SEGMENTS].numberOfMeasures++;
			segments[MAX_SEGMENTS].measures[++data_counter] = h_data[i];
		}
		else
		{
			//point belongs to other object -> store current Segment and reset tmp-segment object
			//only keep segments with at least 3 points
			if(segments[MAX_SEGMENTS].numberOfMeasures >= 3)
			{
				segments[segment_counter].numberOfMeasures = segments[MAX_SEGMENTS].numberOfMeasures;
				for(int j=0; j<segments[MAX_SEGMENTS].numberOfMeasures; j++)
				{
					segments[segment_counter].measures[j] = segments[MAX_SEGMENTS].measures[j];
				}
				segment_counter++;
			}
			segments[MAX_SEGMENTS].numberOfMeasures = 1;
			segments[MAX_SEGMENTS].measures[0] = h_data[i];
			data_counter = 0;
		}
	}

	if(segments[MAX_SEGMENTS].numberOfMeasures >= 3)
	{
		segments[segment_counter].numberOfMeasures = segments[MAX_SEGMENTS].numberOfMeasures;
		for(int j=0; j<segments[MAX_SEGMENTS].numberOfMeasures; j++)
		{
			segments[segment_counter].measures[j] = segments[MAX_SEGMENTS].measures[j];
		}
		++segment_counter;
	}

	printf("Extracted %d Objects from Laserdata\n", segment_counter);

	error = cudaMemcpy(d_rawSegs, segments.data(), segment_counter*sizeof(raw_segment), cudaMemcpyHostToDevice);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}

	for(int i=0; i<segment_counter;i++)
	{
		error = cudaMemcpy(&((d_rawSegs+i)->measures), &(d_rawMeasure.data()[i]), sizeof(laserdata_raw*), cudaMemcpyHostToDevice);
		if (error != cudaSuccess) {
			printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
					cudaGetErrorString(error), error, __LINE__);
		}
		error = cudaMemcpy(d_rawMeasure[i], segments.at(i).measures, segments.at(i).numberOfMeasures*sizeof(laserdata_raw), cudaMemcpyHostToDevice);
		if (error != cudaSuccess) {
			printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
					cudaGetErrorString(error), error, __LINE__);
		}
	}
	cudaEventRecord(stopEvent, 0);
	cudaEventSynchronize(stopEvent);
	cudaEventElapsedTime(&time, startEvent, stopEvent);
	std::cout << "segment data Time: " << time << std::endl;
	coordinateTransform<<<segment_counter,NUMBER_LASERRAYS>>>(d_rawSegs, d_carSegs);
	std::vector<cartesian_segment> transformedData(segment_counter);
	cudaDeviceSynchronize();
	cudaEventRecord(startEvent, 0);
	error = cudaMemcpy(transformedData.data(), d_carSegs, segment_counter*sizeof(cartesian_segment), cudaMemcpyDeviceToHost);
	if (error != cudaSuccess) {
		printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
				cudaGetErrorString(error), error, __LINE__);
	}
	std::vector<laserdata_cartesian*> memory(segment_counter);
	for(uint i=0; i<segment_counter; i++)
	{
		memory.at(i) = (laserdata_cartesian*) malloc(transformedData.at(i).numberOfMeasures*sizeof(laserdata_cartesian));
		transformedData.at(i).measures = memory.at(i);
		error = cudaMemcpy(transformedData.at(i).measures, d_carMeasure[i], transformedData.at(i).numberOfMeasures*sizeof(laserdata_cartesian), cudaMemcpyDeviceToHost);
		if (error != cudaSuccess) {
			printf("cudaGetDevice returned error %s (code %d), line(%d)\n",
					cudaGetErrorString(error), error, __LINE__);
		}
	}
/*	unsigned int x = 0;
	unsigned long long max = 1000000;
	cudaMemcpy(d_minDistance, &max, sizeof(unsigned long long), cudaMemcpyHostToDevice);
	getRelevantMeas<<<segment_counter, NUMBER_LASERRAYS>>>(d_carSegs,d_carLaser,d_minDistance);
	cudaMemcpy(d_index, &x, sizeof(unsigned int), cudaMemcpyHostToDevice);
	extractVehicles<<<segment_counter,1>>>(d_carLaser,d_vehicles, d_index, currentSpeed, currentYawRate);
	cudaMemcpy(&x,d_index, sizeof(unsigned int), cudaMemcpyDeviceToHost);*/
	//visualizer.visualizeSegmentsAsPointCloud(transformedData,number);
	std::vector<PointCellDevice> vehicles = computeVehicleState(transformedData, number);
//	for(uint i=0; i<x;i++)
//	{
//		cudaMemcpy(vehicles.data(),d_vehicles, x*sizeof(PointCellDevice), cudaMemcpyDeviceToHost);
//	}
	cudaEventRecord(stopEvent, 0);
	cudaEventSynchronize(stopEvent);
	cudaEventElapsedTime(&time, startEvent, stopEvent);
//	std::cout << "extraction data Time: " << time << std::endl;
//	std::cout << "Extracted "  << x << " cars from data" << std::endl;
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

//	C2= this->computeEuclideanDistance(p1,p2)/p1.distance;
	C1 = sqrt(2*(1-cos(deltaAlpha)));

	return C0 + C1*min_distance;

//	double beta = 0.5;
}

std::vector<cartesian_segment> DataReader::doCoordinateTransform(std::vector<raw_segment> segments)
{
	/*std::vector<cartesian_segment> transformedData;

	for(uint i = 0; i<segments.size(); i++)
	{
		raw_segment seg = segments.at(i);
		cartesian_segment curSeg;
		curSeg.numberOfMeasures = seg.numberOfMeasures;

		double angleInRadians;

		for(int j=0; j<seg.numberOfMeasures; j++)
		{
			laserdata_raw raw = seg.measures.at(j);
			laserdata_cartesian currentLaser;

			angleInRadians = raw.angle * M_PI / 180.0;

			currentLaser.x = raw.distance*cos(angleInRadians);
			currentLaser.y = raw.distance*sin(angleInRadians);



		//	curSeg.measures.push_back(currentLaser);
		}
		transformedData.push_back(curSeg);
	}
	return transformedData;*/
}

std::vector<PointCellDevice> DataReader::computeVehicleState(std::vector<cartesian_segment> segments, std::string number)
{
	std::vector<PointCellDevice> vehicles;


	cartesian_segment currentSegment;
	std::vector<laserdata_cartesian> relevantPoints;
	std::vector<std::vector<laserdata_cartesian> > toPlot;

	for(uint i=0; i<segments.size(); i++)
	{
		currentSegment = segments.at(i);
		relevantPoints = getRelevantMeasuresFromSegment(currentSegment);

		//ignore guard railing
/*		if(relevantPoints.at(0).x == relevantPoints.at(1).x
				&& relevantPoints.at(1).y == relevantPoints.at(1).y)
		{
			continue;
		}
		else if(relevantPoints.at(1).x == relevantPoints.at(2).x
				&& relevantPoints.at(1).y == relevantPoints.at(2).y)
		{
			continue;
		}*/

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
	/*	std::cout << i << std::endl;
		std::cout << "left x = " << relevantPoints[0].x << " y = " << relevantPoints[0].y << std::endl;
		std::cout << "nearest x = " << relevantPoints[1].x << " y = " << relevantPoints[1].y << std::endl;
		std::cout << "right x = " << relevantPoints[2].x << " y = " << relevantPoints[2].y << std::endl;*/

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
			PointCellDevice vehicle;
			//vehicle.width = width;
			double y = relevantPoints[left].y + width/2;
			vehicle.setX(relevantPoints[left].x + length/2);//x
			vehicle.setY(relevantPoints[left].y + width/2); // y
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
			vehicle.setTheta(theta*M_PI / 180.0); //theta
			//due to prior knowledge on highways, velocitys for diffrent lanes are estimated as below
			if(y < -4.5)
			{
				vehicle.setVelocity(currentSpeed + 11.11); //velocity + 40kmh
			}
			else if(y < -1.5)
			{
				vehicle.setVelocity(currentSpeed + 5.55); //velocity + 20kmh
			}
			else if(y > 4.5)
			{
				vehicle.setVelocity(currentSpeed - 11.11); //velocity - 40kmh
			}
			else if(y > 1.5)
			{
				vehicle.setVelocity(currentSpeed - 5.55); //velocity - 20kmh
			}
			else
			{
				vehicle.setVelocity(currentSpeed); //velocity
			}
			vehicle.setPhi(currentYawRate); //yaw rate
			vehicle.subInvtl = 0.5;
			vehicles.push_back(vehicle);
			toPlot.push_back(relevantPoints);
		}
	}
	std::cout<<"Extracted " << toPlot.size() << " Vehicles from Data" << std::endl;
	visualizer.visualizeVehiclesAsRectangle(toPlot, number);
	return vehicles;
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


