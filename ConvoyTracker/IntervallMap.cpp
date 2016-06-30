/*
 * IntervallMap.cpp
 *
 *  Created on: 27.06.2016
 *      Author: Sebastian Reinhart
 */

#include "IntervallMap.h"

IntervallMap::IntervallMap() {
	intervalLength = INTERVALL_LENGTH;
	numberOfIntervalls = NUMBER_OF_INTERVALS;
	xSubIntervall = 0;
	allocateIntervallMap();
}

IntervallMap::IntervallMap(int numberOfIntervals, int length)
{
	intervalLength = length;
	numberOfIntervalls = numberOfIntervals;
	xSubIntervall = 0;
	allocateIntervallMap();
}

IntervallMap::~IntervallMap() {
	freeIntervallMap();
}

void IntervallMap::allocateIntervallMap() {
	this->map = new std::set<PC>*[numberOfIntervalls];
	for (int i = 0; i < numberOfIntervalls; i++) {
		map[i] = new std::set<PC>();
	}
}

void IntervallMap::freeIntervallMap() {
	for (int i = 0; i < numberOfIntervalls; i++) {
		delete map[i];
	}
	delete[] this->map;
}

/**
 * 0|				|
 * 1|				|
 * 2|				|
 * 3|				|
 * 4|				|
 * 5|				|
 * 6|				|
 * 7|				|
 * 		   ||
 *
 * shifts the whole structure by the number of intervalls that are coverd within @param xMotion and the already
 * stored x-position in intervall to the bottom to compensate ego motion
 */
void IntervallMap::shiftStructure(double xMotion) {
	xSubIntervall += xMotion;
	int numIntervalls = (int)(xSubIntervall - intervalLength);
	xSubIntervall -= numIntervalls;
	for (int i = 0; i < numIntervalls; i++) {
		map[numberOfIntervalls - 1]->clear();
		std::set<PC>* tmp = map[numberOfIntervalls - 1];
		for (int j = numberOfIntervalls - 1; j >= 1; j--) {
			map[j] = map[j - 1];
		}
		map[0] = tmp;
	}
}
/*
 * rotates the whole map by @param angle and shifts the PCs by @param yMotion
 */
void IntervallMap::rotateStructure(double angle, double yMotion) {
	std::map<int,std::vector<std::set<PC>::iterator> >ups;
	std::vector<std::set<PC>::iterator> down;
	//1.Step correct directions of stored PCs
	for(int j = 0; j<numberOfIntervalls; j++)
	{
		std::vector<std::set<PC>::iterator> up;
		for (std::set<PC>::iterator it=map[j]->begin(); it!=map[j]->end(); ++it)
		{
			(*it).y = (*it).y -yMotion;
			(*it).theta -= angle;
			//2. compensate rotation
			double xAbs = (numberOfIntervalls -j) * intervalLength - xSubIntervall;
			double yAbs = (*it).y;

			double angleInRadians = angle *M_PI /180;
			double mat[2][2] = {{cos(angleInRadians),-sin(angleInRadians)}, {sin(angleInRadians), cos(angleInRadians)}};
			xAbs = (mat[0][0]*xAbs + mat[0][1]*yAbs) - xAbs;
			yAbs = (mat[1][0]*xAbs + mat[1][1]*yAbs) - yAbs;

			(*it).y -= yAbs;
			//(*it).x -= xAbs;
			if(xAbs > 0.5 * intervalLength )
			{
				//move one intervall up
				up.push_back(it);
			}
			else if(xAbs < -0.5 * intervalLength)
			{
				//move one intervall down;
				down.push_back(it);
			}
		}
		for(int i = 0; i< up.size(); i++)
		{
			map[j]->erase(up.back());
		}
		if(j+1 < numberOfIntervalls)
		{
			ups.insert(std::pair<int, std::vector<std::set<PC>::iterator> > (j+1, up));
		}
		for(int i = 0; i< down.size(); i++)
		{
			map[j]->erase(down.back());
			if(j > 0)
			{
				map[j-1]->insert(*down.back());
			}
			down.pop_back();
		}
	}
}
