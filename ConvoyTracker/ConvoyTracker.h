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
#include "DataReader.h"
#include "data.h"
#include "IntervalMap.h"
#include "PGMReader.h"
#include "PointCell.h"
#include "DataVisualizer.h"


#define ASSOCIATION_THRESHOLD 4


/**
 * Szenario 1: own car driving straight with 120km/h in the second of three lanes, all other cars are driving with the same speed as well
 * 			   Convoy should be tracked on the left lane
 * Szenario 2: own car driving straight with 120km/h in the second of three lanes, two cars in the left lane are driving straight with 140 km/h,
 * 			   but there is a obstacle in this lane so the cars move to the middle lane to avoid contact
 * Szenario 3: own car driving straight with 120km/h in the second of three lanes, two cars in the left lane are driving straight with 140 km/h,
 * 			   but there is a obstacle in this lane so the cars move to the middle lane to avoid contact but move back to the left lane behind this obstacle
 * Szenario 4: own car is changing the lane
 *
 */



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
};

#endif /* CONVOYTRACKER_H_ */
