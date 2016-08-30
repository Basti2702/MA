/*
 * data.h
 *
 *  Created on: 06.06.2016
 *      Author: Sebastian Reinhart
 */

#ifndef DATA_H_
#define DATA_H_

#include <vector>

#define NUMBER_LASERRAYS 581
#define CARINTERVAL 5
#define TIMESTAMP 0.04 //40ms -> 25hz
#define NUMBER_OF_INTERVALS 100
#define INTERVALL_LENGTH 1 //unit m

#define CREATE_MEASURES

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
#define SZENARIO 4

#if SZENARIO == 1
	#define NUM_MEASUREMENT 100
	#define MEASUREPATH "./Laserdata/Szenario1/Measure/LaserMessung"
	#define EMLPATH "./Laserdata/Szenario1/EML/EML"
	#define VISUALIZATIONPATH "./Visualization/Szenario1"
#elif SZENARIO == 2
	#define NUM_MEASUREMENT 41
	#define MEASUREPATH "./Laserdata/Szenario2/Measure/LaserMessung"
	#define EMLPATH "./Laserdata/Szenario2/EML/EML"
	#define VISUALIZATIONPATH "./Visualization/Szenario2"
#elif SZENARIO == 3
	#define NUM_MEASUREMENT 60
	#define MEASUREPATH "./Laserdata/Szenario3/Measure/LaserMessung"
	#define EMLPATH "./Laserdata/Szenario3/EML/EML"
	#define VISUALIZATIONPATH "./Visualization/Szenario3"
#elif SZENARIO == 4
	#define NUM_MEASUREMENT 100
	#define MEASUREPATH "./Laserdata/Szenario4/Measure/LaserMessung"
	#define EMLPATH "./Laserdata/Szenario4/EML/EML"
	#define VISUALIZATIONPATH "./Visualization/Szenario4"
#endif



typedef struct laserdata_raw{
	double angle;
	double distance;
	int valid;
}laserdata_raw_array[NUMBER_LASERRAYS];

struct laserdata_cartesian{
	double x;
	double y;
};

struct raw_segment{
	int numberOfMeasures;
	std::vector<laserdata_raw> measures;
};

struct cartesian_segment{
	int numberOfMeasures;
	std::vector<laserdata_cartesian> measures;
};

typedef struct EMLPos{
	double x;
	double y;
	double theta;
	double subIntvl;
} Pos;

typedef struct Convoy{
	int ID;
	std::vector<int> participatingVehicles;
	std::vector<EMLPos> tracks;
} Convoy;



#endif /* DATA_H_ */
