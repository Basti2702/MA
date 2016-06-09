/*
 * DataReader.cpp
 *
 *  Created on: 06.06.2016
 *      Author: Sebastian Reinhart
 */

#include "DataReader.h"
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <assert.h>
#include <stdlib.h>
#include <math.h>

DataReader::DataReader() {
	// TODO Auto-generated constructor stub

}

DataReader::~DataReader() {
	// TODO Auto-generated destructor stub
}

/*
 * Reads the data out of the specified file and writes it to given array
 * @param
 * 	data: array for raw laserdata
 * @return
 * 	1 if an error occurs
 * 	0 if everything is ok
 */
int DataReader::getLaserData(laserdata_raw_array data)
{
    std::ifstream input("/home/basti/MA/ConvoyTracker/Laserdata/laser1Messung.txt");
    std::string line;
    int counter = 0;
	std::string segment;
	/*
	 * Laserscanner range 145° with resolution of 0.25° -> 580 values + Zero value = 581
	 *-72°  0°    72°
	 * \    |    /
	 *  \   |   /
	 *   \  |  /
	 *    \ | /
	 *     \|/
	 *    Laser
	 */
	double angle = -72.5;

    //Skip first 581 lines, just read out the second level of datas
    while(counter < NUMBER_LASERRAYS && std::getline( input, line )) {
    	++counter;
  //  	std::cout<< counter <<'\n';
    }

    //now read the data we are interested in
    counter = 0;
    while( std::getline( input, line ) && counter < NUMBER_LASERRAYS ) {
    	//std::cout<<line<<'\n';
    	std::stringstream ss;

    	ss << line;

    	int datacnt = 1;
    	//extract relevant data from line
    	while(std::getline(ss, segment, ' '))
    	{
    		if(segment.size() > 0)
    		{
    			if(segment.at(0) < 48 || segment.at(0) > 57)
    			{
    				continue;
    			}
    			else
    			{
    				if(datacnt == 5)
    				{
    					data[counter].valid = atoi(segment.c_str());
    				}
    				else if(datacnt == 6)
    				{
    					data[counter].distance = atof(segment.c_str());
    				}
    		//		std::cout<<segment << ' ';
            		++datacnt;
    			}
    		}
    	}
    	data[counter].angle = angle;
    	std::cout<< "Angle: " << data[counter].angle << " Distance: " << data[counter].distance << " Valid: " << data[counter].valid << '\n';
    	angle += 0.25;
    	++counter;
    }
    //assert(angle == 72.5);
    return 0;
}
/**
 * Runs over all laser points and tries to group them to segments, regarding to their euclidean distance to their neighbor point
 * going from left to right
 */
int DataReader::processLaserData(laserdata_raw data[], int numElements)
{
	laserdata_raw currentMeasure;
	laserdata_raw oldMeasure = data[0];
	std::vector<raw_segment> segments;
	raw_segment currentSegment;

	//first point automatically is part of the first segment;
	currentSegment.numberOfMeasures = 1;
	currentSegment.measures.push_back(oldMeasure);

	//iterate over all measures
	for(int i=0; i<numElements; ++i)
	{
		currentMeasure = data[i];
		if(computeEuclideanDistance(oldMeasure, currentMeasure) <= computeThreshold(oldMeasure, currentMeasure))
		{
			//add current point in existing segment
			currentSegment.numberOfMeasures++;
			currentSegment.measures.push_back(currentMeasure);
		}
		else
		{
			//point belongs to other object -> store current Segment and reset tmp-segment object
			//only keep segments with at least 3 points
			if(currentSegment.numberOfMeasures >= 3)
			{
				segments.push_back(currentSegment);
			}
			currentSegment.numberOfMeasures = 0;
			currentSegment.measures.clear();
		}
		oldMeasure = currentMeasure;
	}

	return 0;
}

/*
 * ANMERKUNG: KÖNNTE FÜR GESAMTES ARRAY PARALLEL BERECHNET WERDEN!!
 *
 */
double DataReader::computeEuclideanDistance(laserdata_raw p1, laserdata_raw p2)
{
	double square1 = p1.distance*p1.distance;
	double square2 = p2.distance*p2.distance;
	double deltaAlpha = p2.angle-p1.angle;
	return sqrt(square1+square2-2*p1.distance*p2.distance*cos(deltaAlpha));
}

double DataReader::computeThreshold(laserdata_raw p1, laserdata_raw p2)
{
	//https://www.researchgate.net/publication/243773062_Model_Based_Object_Classification_and_Tracking_in_Traffic_Scenes_from_Range_Images
	double C0 = 0.5;
	double C1;
	double min_distance = p2.distance;

	if(p1.distance < p2.distance)
	{
		min_distance = p1.distance;
	}

	C1 = this->computeEuclideanDistance(p1,p2)/p1.distance;

	return C0 + C1*min_distance;

//	double beta = 0.5;

}
