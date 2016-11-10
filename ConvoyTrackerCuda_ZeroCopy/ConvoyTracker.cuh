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
#include <limits.h>
#include "DataReader.cuh"
#include "data.cuh"
#include "PGMReader.cuh"
#include "PointCellDevice.cuh"
#include "DataVisualizer.cuh"

#include <cuda.h>

#define ASSOCIATION_THRESHOLD 4

class ConvoyTracker {
public:
	ConvoyTracker();
	virtual ~ConvoyTracker();

	DataReader reader;
//	std::vector<PointCellDevice> intervalMap;
	PointCellDevice* h_intervalMap;
	PointCellDevice* d_intervalMap_ptr;
	int intervalSize;

	//pointer for device
	History* d_history_ptr;
	PointCellDevice* d_newVeh_ptr;
	PointCellDevice* h_convoyCheck;
	int convoyCheckSize;
	int* d_historyMatch_ptr;
	int* h_historyMatch;
	int* h_historyMatchSelf;
	int* d_historyMatchSelf_ptr;
	Convoy* d_convoys_ptr;

	double* xSubInterval;
	double* d_subIntvl_ptr;

	History* history;
	int startIndexHistory;
	int endIndexHistory;
	int historySize;

	Convoy* convoys;
	int startIndexConvoys;
	int endIndexConvoys;
	int convoySize;


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
	void transformDataFromDevice();
	void findConvoySelf(int ID);
private:
	int ID;
	int convoyID;

	double currentSpeed;
	double currentYawRate;

	double x, xOld;
	double y, yOld;
	double yaw, yawOld;

	bool currentHistoryOnDevice;
	bool currentConvoyOnDevice;
	DataVisualizer visualizer;


	std::vector<EMLPos> EML;

	bool checkConvoyForDuplicate(double x, Convoy c);
	bool checkHistoryForDuplicate(double x, int historyIndex);
	int findIDinConvoy(Convoy c, int id);
	int findHistoryWithID(int id);

};

#endif /* CONVOYTRACKER_H_ */
