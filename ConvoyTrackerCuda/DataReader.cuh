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

	std::vector<PointCellDevice> processLaserData(std::string number, double currentSpeed, double currentYawRate);

private:
	DataVisualizer visualizer;
	double currentSpeed;
	double currentYawRate;
	cartesian_segment* d_carSegs;
	raw_segment* d_rawSegs;
	laserdata_cartesian* d_carLaser;
	std::vector<laserdata_cartesian*> d_carMeasure;
	std::vector<laserdata_raw*> d_rawMeasure;

	laserdata_raw* d_data;
	double* d_dist;
	double* d_thresh;
	int* d_numSegments;

	std::vector<raw_segment> segments;

	void readEMLData(std::string number);
	int getLaserData(laserdata_raw_array data, std::string number);
	double computeEuclideanDistance(laserdata_raw p1, laserdata_raw p2);
	double computeThreshold(laserdata_raw p1, laserdata_raw p2);
	std::vector<PointCellDevice> computeVehicleState(std::vector<cartesian_segment> segments, std::string number);
	std::vector<cartesian_segment> doCoordinateTransform(std::vector<raw_segment> segments);
	std::vector<laserdata_cartesian> getRelevantMeasuresFromSegment(cartesian_segment segment);
};

#endif /* DATAREADER_H_ */
