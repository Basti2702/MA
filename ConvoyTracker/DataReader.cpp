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
    std::ifstream input("/home/basti/MA/ConvoyTracker/Laserdata/LaserMessung0031.txt");
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
    int lineCounter = 0;
    while( std::getline( input, line ) && lineCounter < NUMBER_LASERRAYS ) {
    	//std::cout<<line<<'\n';
    	std::stringstream ss;

    	ss << line;

    	int datacnt = 1;
    	//extract relevant data from line
    	int valid = 0;
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
    					valid = atoi(segment.c_str());
    					if(valid == 1)
    					{
    						data[counter].valid = valid;
    					}
    				}
    				else if(datacnt == 6 && valid == 1)
    				{
    					data[counter].distance = atof(segment.c_str());
    			    	break;
    				}
    		//		std::cout<<segment << ' ';
            		++datacnt;
    			}
    		}
    	}
    	if(valid)
    	{
    		data[counter].angle = angle;
    		std::cout<< "Angle: " << data[counter].angle << " Distance: " << data[counter].distance << " Valid: " << data[counter].valid << '\n';
	    	++counter;
    	}
    	angle += 0.25;
    	++lineCounter;
    }
    std::cout << counter << std::endl;
    return counter;
}
/**
 * Runs over all laser points and tries to group them to segments, regarding to their euclidean distance to their neighbor point
 * going from left to right
 */
int DataReader::processLaserData()
{
	laserdata_raw data[NUMBER_LASERRAYS];
	laserdata_raw currentMeasure;
	laserdata_raw oldMeasure;
	std::vector<raw_segment> segments;
	raw_segment currentSegment;

	//read new data from file
	int numElements = getLaserData(data);

	oldMeasure = data[0];


	//first point automatically is part of the first segment;
	currentSegment.numberOfMeasures = 1;
	currentSegment.measures.push_back(oldMeasure);

	//iterate over all measures
	for(int i=1; i<numElements; i++)
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
			std::cout << "Started new Segment oldMeasure.angle: " << oldMeasure.angle << " oldMeasure.distance: " << oldMeasure.distance << " currentMeasure.angle: " << currentMeasure.angle << " currentMeasure.distance: " << currentMeasure.distance << " Euclidean Distance: " << computeEuclideanDistance(oldMeasure, currentMeasure) << " Threshold: " << computeThreshold(oldMeasure, currentMeasure) << std::endl;
			currentSegment.numberOfMeasures = 1;
			currentSegment.measures.clear();
			currentSegment.measures.push_back(currentMeasure);
		}
		oldMeasure = currentMeasure;
	}

	if(currentSegment.numberOfMeasures >= 3)
	{
		segments.push_back(currentSegment);
	}

	std::cout << "Extracted " << segments.size() << "Objects from Laserdata" << std::endl;

	std::vector<cartesian_segment> transformedData = doCoordinateTransform(segments);
	visualizer.visualizeSegmentsAsPointCloud(transformedData);
	std::vector<PC> vehicles = computeVehicleState(transformedData);
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
	deltaAlpha = (deltaAlpha * M_PI / 180.0);
	return sqrt(square1+square2-2*p1.distance*p2.distance*cos(deltaAlpha));
}

double DataReader::computeThreshold(laserdata_raw p1, laserdata_raw p2)
{
	//https://www.researchgate.net/publication/243773062_Model_Based_Object_Classification_and_Tracking_in_Traffic_Scenes_from_Range_Images
	double C0 = 1.0;
	double C1, C2;
	double min_distance = p2.distance;

	double deltaAlpha = p2.angle-p1.angle;
	deltaAlpha = (deltaAlpha * M_PI / 180.0);

	if(p1.distance < p2.distance)
	{
		min_distance = p1.distance;
	}

//	C2= this->computeEuclideanDistance(p1,p2)/p1.distance;
	C1 = sqrt(2*(1-cos(deltaAlpha)));

	return C0 + C1*min_distance;

//	double beta = 0.5;
}

std::vector<cartesian_segment> DataReader::doCoordinateTransform(std::vector<raw_segment> segments)
{
	std::vector<cartesian_segment> transformedData;

	for(uint i = 0; i<segments.size(); i++)
	{
		raw_segment seg = segments.at(i);
		cartesian_segment curSeg;
		curSeg.numberOfMeasures = seg.numberOfMeasures;

		double angleInRadians;

		for(int j=0; j<seg.numberOfMeasures; j++)
		{
			laserdata_raw raw = seg.measures.at(j);
			laserdata_cartesian currentLaser;

			angleInRadians = raw.angle * M_PI / 180.0;

			currentLaser.x = raw.distance*cos(angleInRadians);
			currentLaser.y = raw.distance*sin(angleInRadians);



			curSeg.measures.push_back(currentLaser);
		}
		transformedData.push_back(curSeg);
	}
	return transformedData;
}

std::vector<PC> DataReader::computeVehicleState(std::vector<cartesian_segment> segments)
{
	std::vector<PC> vehicles;
	for(uint i=0; i<segments.size(); i++)
	{
		cartesian_segment currentSeg = segments.at(i);


	}
}
