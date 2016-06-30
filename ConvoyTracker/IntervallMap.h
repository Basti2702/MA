/*
 * IntervallMap.h
 *
 *  Created on: 27.06.2016
 *      Author: Sebastian Reinhart
 */

#ifndef INTERVALLMAP_H_
#define INTERVALLMAP_H_

#include <set>
#include <math.h>
#include <map>
#include "data.h"

#define NUMBER_OF_INTERVALS 10
#define INTERVALL_LENGTH 5

class IntervallMap {
public:
	IntervallMap();
	IntervallMap(int numberOfIntervals, int length);
	virtual ~IntervallMap();

	void shiftStructure(double xMotion);
	void rotateStructure(double angle, double yMotion);

private:
	void allocateIntervallMap();
	void freeIntervallMap();

	int numberOfIntervalls;
	int intervalLength;
	double xSubIntervall;
	std::set<PC>** map;
};

#endif /* INTERVALLMAP_H_ */
