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
#include <exception>
#include <map>
#include <algorithm>
#include <chrono>
#include "DataReader.h"
#include "data.h"
#include "IntervalMap.h"
#include "PGMReader.h"
#include "PointCell.h"
#include "DataVisualizer.h"


#define ASSOCIATION_THRESHOLD 4
#define ASSOCIATION_Y_THRESHOLD 2

class ConvoyTracker {
public:
	ConvoyTracker();
	virtual ~ConvoyTracker();

	DataReader reader;
	IntervalMap intervalMap;



	void readEMLData(std::string number);
	void associateAndUpdate(std::vector<PointCell> vehicles, std::vector<pcNode*> trackedVehicles);
	void findConvoy(PointCell vehicle);
	void shiftConvoyHistory(double x);
	void rotateConvoyHistory(double theta, double y);
	void visualizeConvoys();
	void visualizeHistory();

	double getCurrentSpeed() const;
	void setCurrentSpeed(double currentSpeed);
	double getCurrentYawRate() const;
	void setCurrentYawRate(double currentYawRate);
	double getX() const;
	void setX(double x);
	double getYOld() const;
	void setYOld(double old);
	double getY() const;
	void setY(double y);
	double getYaw() const;
	void setYaw(double yaw);
	double getYawOld() const;
	void setYawOld(double yawOld);
	double getXOld() const;
	void setXOld(double old);

private:
	int ID;
	int convoyID;

	double currentSpeed;
	double currentYawRate;

	double x, xOld;
	double y, yOld;
	double yaw, yawOld;

	DataVisualizer visualizer;

	std::map<int, std::vector<PointCell> > history;
	std::vector<Convoy>convoys;
	std::vector<Pos> EML;

	bool checkConvoyForDuplicate(double x, Convoy c);
};

#endif /* CONVOYTRACKER_H_ */
