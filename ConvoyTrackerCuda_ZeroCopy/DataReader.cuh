/*
 * DataReader.h
 *
 *  Created on: 06.06.2016
 *      Author: Sebastian Reinhart
 */

#ifndef DATAREADER_H_
#define DATAREADER_H_

#include "data.cuh"
#include "DataVisualizer.cuh"
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <assert.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <limits.h>
#include <cuda.h>
#include "PointCell.cuh"
#include "PointCellDevice.cuh"


class DataReader {
public:
	DataReader();
	virtual ~DataReader();

	int processLaserData(std::string number, double currentSpeed, double currentYawRate, PointCellDevice* h_vehicles);

private:
	DataVisualizer visualizer;
	double currentSpeed;
	double currentYawRate;
	cartesian_segment* d_carSegs_ptr;
	raw_segment* d_rawSegs_ptr;
	unsigned long long* d_minDistance_ptr;
	unsigned int* d_index_ptr;

	unsigned int* h_index;
	unsigned long long* h_minDistance;
	laserdata_raw* d_data_ptr;
	laserdata_raw* h_data;
	laserdata_cartesian* h_relMeas;
	laserdata_cartesian* d_relMeas_ptr;
	double* d_dist_ptr;
	double* d_thresh_ptr;

	double* dist;
	double* thresh;

	cudaStream_t stream0, stream1;

	raw_segment* raw_segments;
	cartesian_segment* car_segments;

	void readEMLData(std::string number);
	int getLaserData(laserdata_raw_array data, std::string number);
	double computeEuclideanDistance(laserdata_raw p1, laserdata_raw p2);
	double computeThreshold(laserdata_raw p1, laserdata_raw p2);
	int computeVehicleState(cartesian_segment* segments, int segmentCounter, std::string number, PointCellDevice* h_vehicles);
	std::vector<cartesian_segment> doCoordinateTransform(std::vector<raw_segment> segments);
	std::vector<laserdata_cartesian> getRelevantMeasuresFromSegment(cartesian_segment segment);
};

#endif /* DATAREADER_H_ */
