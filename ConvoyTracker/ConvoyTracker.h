/*
 * ConvoyTracker.h
 *
 *  Created on: 06.06.2016
 *      Author: basti
 */

#ifndef CONVOYTRACKER_H_
#define CONVOYTRACKER_H_

#include <string>
#include <iostream>
#include "DataReader.h"
#include "data.h"
#include "IntervalMap.h"
#include "PGMReader.h"
#include "PointCell.h"

#define NUM_MEASUREMENT 10
#define ASSOCIATION_THRESHOLD 3.5
//1242
class ConvoyTracker {
public:
	ConvoyTracker();
	virtual ~ConvoyTracker();

	DataReader reader;
	IntervalMap intervalMap;

	double currentSpeed;
	double currentYawRate;

	void readEMLData(std::string number);
	void associateAndUpdate(std::vector<PointCell> vehicles, std::vector<pcNode*> trackedVehicles);
private:

};

#endif /* CONVOYTRACKER_H_ */
