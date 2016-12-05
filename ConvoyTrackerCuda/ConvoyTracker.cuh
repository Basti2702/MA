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
	PointCellDevice* d_intervalMap;
	int intervalSize;

	//pointer for device
	History* d_history;
	PointCellDevice* d_newVeh;
	PointCellDevice* h_convoyCheck;
	PointCellDevice* h_vehicles;
	PointCellDevice* d_vehicles;
	int convoyCheckSize;
	int* d_historyMatch;
	int* h_historyMatch;
	int* h_historyMatchSelf;
	int* d_historyMatchSelf;
	Convoy* d_convoys;

	float* xSubInterval;
	float* d_subIntvl_ptr;
	int* h_IDincluded;
	int* d_IDincluded;
	bool* h_duplicate;
	bool* d_duplicate;
	float* h_updateData;
	float* d_updataData;
	int* h_intvlIndex;
	int* d_intvlIndex;
	float* h_distance;
	float* d_distance;

	History* history;
	int startIndexHistory;
	int endIndexHistory;
	int historySize;

	Convoy* convoys;
	int startIndexConvoys;
	int endIndexConvoys;
	int convoySize;

	PointCellDevice* h_vehicleSim;
	PointCellDevice* d_vehicleSim;

	cudaStream_t stream2, stream3, stream4, stream5;

	void readEMLData(std::string number);
	void associateAndUpdate(int vehicleCount, std::vector<PointCellDevice*> trackedVehicles);
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

	bool checkConvoyForY(float y, float x, Convoy c);

};

#endif /* CONVOYTRACKER_H_ */
