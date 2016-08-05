/*
 * DataReader.h
 *
 *  Created on: 06.06.2016
 *      Author: Sebastian Reinhart
 */

#ifndef DATAREADER_H_
#define DATAREADER_H_

#include "data.h"
#include "DataVisualizer.h"
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <assert.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>
#include "PointCell.h"


class DataReader {
public:
	DataReader();
	virtual ~DataReader();

	std::vector<PointCell> processLaserData(std::string number);

private:
	DataVisualizer visualizer;
	int ID;


	int getLaserData(laserdata_raw_array data, std::string number);
	double computeEuclideanDistance(laserdata_raw p1, laserdata_raw p2);
	double computeThreshold(laserdata_raw p1, laserdata_raw p2);
	std::vector<PointCell> computeVehicleState(std::vector<cartesian_segment> segments, std::string number);
	std::vector<cartesian_segment> doCoordinateTransform(std::vector<raw_segment> segments);
	std::vector<laserdata_cartesian> getRelevantMeasuresFromSegment(cartesian_segment segment);
};

#endif /* DATAREADER_H_ */
