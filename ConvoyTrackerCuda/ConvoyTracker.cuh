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
#include <limits.h>
#include "DataReader.cuh"
#include "data.cuh"
#include "PGMReader.cuh"
#include "PointCell.cuh"
#include "PointCellDevice.cuh"
#include "DataVisualizer.cuh"

#include <cuda.h>

#define ASSOCIATION_THRESHOLD 4
#define ASSOCIATION_Y_THRESHOLD 2

class ConvoyTracker {
public:
	ConvoyTracker();
	virtual ~ConvoyTracker();

	DataReader reader;
//	IntervalMap intervalMap;
	std::vector<PointCellDevice> intervalMap;
	int setMem[MAX_SEGMENTS];

	//pointer for device
	PointCellDevice* d_history;
	PointCellDevice* d_newVeh;
	int* d_historyMatch;
	int* h_historyMatch;
	EMLPos* d_convoys;
	PointCellDevice* d_intervalMap;

	double* xSubInterval;
	double* d_subIntvl_ptr;

//	std::vector<double*> intervalPointer;
//	std::vector<double*> historyPointer;

	std::map<int, std::vector<PointCellDevice> > history;
	std::vector<Convoy>convoys;

	void readEMLData(std::string number);
	void associateAndUpdate(std::vector<PointCellDevice> vehicles, std::vector<PointCellDevice*> trackedVehicles);
	void findConvoy(PointCellDevice vehicle);
	void shiftConvoyHistory(double x);
	void rotateConvoyHistory(double theta, double y);
	void shiftStructure(double xMotion);
	void rotateStructure(double angle, double yMotion);
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
	void transformDataToDevice();
	void transformDataFromDevice();

private:
	int ID;
	int convoyID;

	double currentSpeed;
	double currentYawRate;

	double x, xOld;
	double y, yOld;
	double yaw, yawOld;

	DataVisualizer visualizer;


	std::vector<Pos> EML;

	bool checkConvoyForDuplicate(double x, Convoy c);
	bool checkHistoryForDuplicate(double x, std::vector<PointCellDevice> c);
};

#endif /* CONVOYTRACKER_H_ */
