/*
 * DataReader.h
 *
 *  Created on: 06.06.2016
 *      Author: Sebastian Reinhart
 */

#ifndef DATAREADER_H_
#define DATAREADER_H_

#include "data.h"

class DataReader {
public:
	DataReader();
	virtual ~DataReader();

	int processLaserData();

private:
	int getLaserData(laserdata_raw_array data);
	double computeEuclideanDistance(laserdata_raw p1, laserdata_raw p2);
	double computeThreshold(laserdata_raw p1, laserdata_raw p2);
	std::vector<PC> computeVehicleState(std::vector<cartesian_segment> segments);
	std::vector<cartesian_segment> doCoordinateTransform(std::vector<raw_segment> segments);
};

#endif /* DATAREADER_H_ */
