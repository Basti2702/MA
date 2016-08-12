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
#define MEASUREPATH "./Laserdata/LaserMessung"
#define EMLPATH "./Laserdata/EML"
#define CARINTERVAL 0
#define TIMESTAMP 0.04 //40ms -> 25hz
#define NUMBER_OF_INTERVALS 100
#define INTERVALL_LENGTH 1 //unit m

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

//Point Cell for IntervalMap
struct PC{
	double y; //middle of rear bumper
	double x; //middle of current Interval
	double vx; //velocity x angular velocity
	double vy; //velocity y
//	double w; //angular velocity see https://en.wikipedia.org/wiki/Angular_velocity
	double theta; //orientation regarding own vehicles direction

	//general Informations, not necessary for kalman filter
	double width; //width of vehicle
//	double length; // lengt of vehicle;
	int ID; //for unique identification
};



#endif /* DATA_H_ */
