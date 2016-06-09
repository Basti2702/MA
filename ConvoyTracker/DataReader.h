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

	int getLaserData(laserdata_raw_array data);
	int processLaserData(laserdata_raw data[], int numElements);
	double computeEuclideanDistance(laserdata_raw p1, laserdata_raw p2);
	double computeThreshold(laserdata_raw p1, laserdata_raw p2);
};

#endif /* DATAREADER_H_ */
