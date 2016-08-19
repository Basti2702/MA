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

//#define CREATE_MEASURES;
#define SZENARIO 1

#if SZENARIO == 1
	#define NUM_MEASUREMENT 100
	#define MEASUREPATH "./Laserdata/Szenario1/LaserMessung"
	#define EMLPATH "./Laserdata/Szenario1/EML"
#elif SZENARIO == 2
	#define NUM_MEASUREMENT 19
	#define MEASUREPATH "./Laserdata/Szenario2/LaserMessung"
	#define EMLPATH "./Laserdata/Szenario2/EML"
#elif SZENARIO == 3
	#define NUM_MEASUREMENT 30
	#define MEASUREPATH "./Laserdata/Szenario3/LaserMessung"
	#define EMLPATH "./Laserdata/Szenario3/EML"
#elif SZENARIO == 4
	#define NUM_MEASUREMENT 30
	#define MEASUREPATH "./Laserdata/Szenario4/LaserMessung"
	#define EMLPATH "./Laserdata/Szenario4/EML"
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
