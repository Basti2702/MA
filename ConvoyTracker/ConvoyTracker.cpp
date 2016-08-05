/*
 * ConvoyTracker.cpp
 *
 *  Created on: 06.06.2016
 *      Author: basti
 */

#include "ConvoyTracker.h"


ConvoyTracker::ConvoyTracker() {
	// TODO Auto-generated constructor stub

}

ConvoyTracker::~ConvoyTracker() {
	// TODO Auto-generated destructor stub
}


std::string getNextMeasureAsString(int i)
{
	std::ostringstream number;
	if(i<10)
	{
		number << "000" << i;
	}
	else if(i<100)
	{
		number << "00" << i;
	}
	else if(i<1000)
	{
		number << "0" << i;
	}
	else
	{
		number << i;
	}
	return number.str();
}

int main()
{
/*	PGMReader pgmReader;
	for(int i=0; i<NUM_MEASUREMENT; i++)
	{
		std::string number = getNextMeasureAsString(i);
		pgmReader.simulateLaserRays(number);
	}*/
	DataReader reader;
	IntervalMap intervalMap;
	PointCell pc;
	pc.stateVector.put(0,0, 10);
	pc.stateVector.put(1,0, 0);
	pc.stateVector.put(2,0, 45*M_PI / 180.0);
	pc.stateVector.put(3,0, 33.33);
	pc.stateVector.put(4,0, 0);
	pc.predict();
	std::cout << "X: " << pc.stateVector.get(0,0) << " Y: " << pc.stateVector.get(1,0) << " Theta: " << pc.stateVector.get(2,0) << " Vel: " << pc.stateVector.get(3,0) << " Phi: " << pc.stateVector.get(4,0) << std::endl;
	PointCell pc2;
	pc2.stateVector.put(0,0, 11.5);
	pc2.stateVector.put(1,0, 1.2);
	pc2.stateVector.put(2,0, 48*M_PI / 180.0);
	pc2.stateVector.put(3,0, 33.33);
	pc2.stateVector.put(4,0, 0);
	pc.update(pc2.stateVector);
	std::cout << "NewState: X: " << pc2.stateVector.get(0,0) << " Y: " << pc2.stateVector.get(1,0) << " Theta: " << pc2.stateVector.get(2,0) << " Vel: " << pc2.stateVector.get(3,0) << " Phi: " << pc2.stateVector.get(4,0) << std::endl;
	std::cout << "StateVec after Update X: " << pc.stateVector.get(0,0) << " Y: " << pc.stateVector.get(1,0) << " Theta: " << pc.stateVector.get(2,0) << " Vel: " << pc.stateVector.get(3,0) << " Phi: " << pc.stateVector.get(4,0) << std::endl;
	std::vector<PointCell> vehicles;
	for(int i=0; i<NUM_MEASUREMENT; i++)
	{
		std::string number = getNextMeasureAsString(i);
		vehicles = reader.processLaserData(number);
		//TODO: iterate over vehicles and assosciate//insert them into the intervalMap
	}
	return 0;
}
