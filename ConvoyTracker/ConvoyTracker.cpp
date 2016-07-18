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
	PC test, test2, test3;
	test.y = 10;
	test2.y = 0;
	test3.y = -5;
	intervalMap.insertPCintoInterval(5, test);
	intervalMap.insertPCintoInterval(5, test);
	std::cout << "---------------------------" << std::endl;
	intervalMap.inorderTracks(5);
	intervalMap.insertPCintoInterval(5, test2);
	intervalMap.inorderTracks(5);
	intervalMap.insertPCintoInterval(5, test3);
	intervalMap.inorderTracks(5);
	intervalMap.deletePCfromInterval(5, test2);
	intervalMap.inorderTracks(5);
	std::vector<PC> vehicles;
	for(int i=0; i<NUM_MEASUREMENT; i++)
	{
		std::string number = getNextMeasureAsString(i);
		vehicles = reader.processLaserData(number);
		//TODO: iterate over vehicles and assosciate//insert them into the intervalMap
	}
	return 0;
}
