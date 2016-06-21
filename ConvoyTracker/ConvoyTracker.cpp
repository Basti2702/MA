/*
 * ConvoyTracker.cpp
 *
 *  Created on: 06.06.2016
 *      Author: basti
 */

#include "ConvoyTracker.h"
#include "DataReader.h"
#include "data.h"

ConvoyTracker::ConvoyTracker() {
	// TODO Auto-generated constructor stub

}

ConvoyTracker::~ConvoyTracker() {
	// TODO Auto-generated destructor stub
}

int main()
{
	DataReader reader;
	for(int i=0; i<NUM_MEASUREMENT; i++)
	{
		reader.processLaserData();
	}
	return 0;
}
