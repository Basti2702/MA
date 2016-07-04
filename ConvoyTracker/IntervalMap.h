/*
 * IntervallMap.h
 *
 *  Created on: 27.06.2016
 *      Author: Sebastian Reinhart
 */

#ifndef INTERVALMAP_H_
#define INTERVALMAP_H_

#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <list>
#include "data.h"

#define NUMBER_OF_INTERVALS 100
#define INTERVALL_LENGTH 1 //unit m

struct node
{
  double key_value;
  std::list<PC>* tracks;
  node *left;
  node *right;
};


class IntervalMap {
public:
	IntervalMap();
	IntervalMap(int numberOfIntervals, int length);
	virtual ~IntervalMap();

	void shiftStructure(double xMotion);
	void rotateStructure(double angle, double yMotion);

private:
	void allocateIntervalMap();
	void freeIntervalMap();
	void insertInterval(node* leaf, double key);
	void destroy_tree(node* leaf);
	node* getInterval(node *leaf, double Intervall);

	int numberOfIntervals;
	int intervalLength;
	double xSubInterval;
	node *map;
};
#endif /* INTERVALMAP_H_ */
