/*
 * DataReader.cpp
 *
 *  Created on: 06.06.2016
 *      Author: Sebastian Reinhart
 */

#include "DataReader.h"


DataReader::DataReader() {
	// TODO Auto-generated constructor stub
	ID = 0;
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
int DataReader::getLaserData(laserdata_raw_array data, std::string number)
{
	std::ostringstream measurePath;
	measurePath << MEASUREPATH << number << ".txt";
	std::cout << measurePath.str() << std::endl;
    std::ifstream input(measurePath.str().c_str());
    std::string line;
    int counter = 0;
	std::string segment;
	/*
	 * Laserscanner range 145° with resolution of 0.25° -> 580 values + Zero value = 581
	 *-72,5° 0°  72,5°
	 * \    |    /
	 *  \   |   /
	 *   \  |  /
	 *    \ | /
	 *     \|/
	 *    Laser
	 */
	double angle = -72.5;

    //Skip first 581 lines, just read out the second level of datas
  /*  while(counter < NUMBER_LASERRAYS && std::getline( input, line )) {
    	++counter;
  //  	std::cout<< counter <<'\n';
    }*/

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
    //		std::cout<< "Angle: " << data[counter].angle << " Distance: " << data[counter].distance << " Valid: " << data[counter].valid << '\n';
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
std::vector<PC> DataReader::processLaserData(std::string number)
{
	laserdata_raw data[NUMBER_LASERRAYS];
	laserdata_raw currentMeasure;
	laserdata_raw oldMeasure;
	std::vector<raw_segment> segments;
	raw_segment currentSegment;

	//read new data from file
	int numElements = getLaserData(data, number);

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
		//	std::cout << "Started new Segment oldMeasure.angle: " << oldMeasure.angle << " oldMeasure.distance: " << oldMeasure.distance << " currentMeasure.angle: " << currentMeasure.angle << " currentMeasure.distance: " << currentMeasure.distance << " Euclidean Distance: " << computeEuclideanDistance(oldMeasure, currentMeasure) << " Threshold: " << computeThreshold(oldMeasure, currentMeasure) << std::endl;
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
	std::cout << "Start PointCloud Visualization" << std::endl;
	visualizer.visualizeSegmentsAsPointCloud(transformedData,number);
	std::cout << "End PointCloud Visualization" << std::endl;
	std::vector<PC> vehicles = computeVehicleState(transformedData, number);

	return vehicles;
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

	double deltaAlpha;
	deltaAlpha = (0.25 * M_PI / 180.0);

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

std::vector<PC> DataReader::computeVehicleState(std::vector<cartesian_segment> segments, std::string number)
{
	std::vector<PC> vehicles;


	cartesian_segment currentSegment;
	std::vector<laserdata_cartesian> relevantPoints;
	std::vector<std::vector<laserdata_cartesian> > toPlot;

	for(uint i=0; i<segments.size(); i++)
	{
		currentSegment = segments.at(i);
		relevantPoints = getRelevantMeasuresFromSegment(currentSegment);

		//ignore guard railing
/*		if(relevantPoints.at(0).x == relevantPoints.at(1).x
				&& relevantPoints.at(1).y == relevantPoints.at(1).y)
		{
			continue;
		}
		else if(relevantPoints.at(1).x == relevantPoints.at(2).x
				&& relevantPoints.at(1).y == relevantPoints.at(2).y)
		{
			continue;
		}*/

		//we have three different points, compute bounds
		PC vehicle;
		//right point - left point
		double width = fabs(relevantPoints[2].y - relevantPoints[0].y);
		double length = fabs(relevantPoints[2].x - relevantPoints[0].x);
		double nearestLengthLeft = fabs(relevantPoints[1].x - relevantPoints[0].x);
		double nearestLengthRight = fabs(relevantPoints[1].x - relevantPoints[2].x);
		double nearestWidthLeft = fabs(relevantPoints[1].y - relevantPoints[0].y);
		double nearestWidthRight = fabs(relevantPoints[1].y - relevantPoints[2].y);
		//compute orientation of object regarding to the driving direction of our own car
		//own direction vector(x,y): (1,0)
	/*	std::cout << i << std::endl;
		std::cout << "left x = " << relevantPoints[0].x << " y = " << relevantPoints[0].y << std::endl;
		std::cout << "nearest x = " << relevantPoints[1].x << " y = " << relevantPoints[1].y << std::endl;
		std::cout << "right x = " << relevantPoints[2].x << " y = " << relevantPoints[2].y << std::endl;*/

		if(length > 2)
		{
			length = fabs(relevantPoints[1].x - relevantPoints[0].x);
			width = fabs(relevantPoints[1].y - relevantPoints[0].y);
		}

		double theta = acos(length/(1*sqrt(width*width + length*length))) * 180.0 / M_PI;

		double thetaLeft = acos(nearestLengthLeft/(1*sqrt(nearestWidthLeft*nearestWidthLeft + nearestLengthLeft*nearestLengthLeft))) * 180.0 / M_PI;

		double thetaRight = acos(nearestLengthRight/(1*sqrt(nearestWidthRight*nearestWidthRight + nearestLengthRight*nearestLengthRight))) * 180.0 / M_PI;

		//objects should not be classified as vehicle if their orientation is bigger than 45°
		//real vehicles should never be rotated over that value

		/*	std::cout << "Theta: " << theta << std::endl;
			std::cout << "ThetaLeft: " << thetaLeft << std::endl;
			std::cout << "ThetaRight: " << thetaRight << std::endl;
			std::cout << "Length: " << length << std::endl;
			std::cout << "Width: " << width << std::endl;*/

		//the detected car probably is defined with the points that form the biggest angle and are wider than 1m
		if(thetaLeft + 5 > theta && nearestWidthLeft > 1)
		{
			theta = thetaLeft;
			length = nearestLengthLeft;
			width = nearestWidthLeft;
		}
		if(thetaRight + 5 > theta && nearestWidthRight > 1)
		{
			theta = thetaRight;
			length = nearestLengthRight;
			width = nearestWidthRight;
		}
		if(theta > 60 && width > 1)
		{

			vehicle.width = width;
			vehicle.ID = ID;
			vehicle.y = relevantPoints[0].y + width/2;
			//vehicle.x = relevantPoints[0].x;
			//vehicle.vx = own speed +- depending on x
			//vehicle.vy = own speed +-
			vehicle.theta = theta;
			vehicle.x = relevantPoints[1].x;//middle x of Interval
			vehicles.push_back(vehicle);
			toPlot.push_back(relevantPoints);
		}
	}
	std::cout<<"Extracted " << toPlot.size() << " Vehicles from Data" << std::endl;
	visualizer.visualizeVehiclesAsRectangle(toPlot, number);
	return vehicles;
}

/**
 * Returns the a vector containing the left border, the nearest point and the right border of a given segment
 */
std::vector<laserdata_cartesian> DataReader::getRelevantMeasuresFromSegment(cartesian_segment segment)
{
	std::vector<laserdata_cartesian> relevantMeasures;
	double leastMeasure = INT_MAX;
	laserdata_cartesian leastLaser;
	//left border is always the first measure in a segment
	relevantMeasures.push_back(segment.measures.at(0));

	//Search for the measure with the least distance to ourself
	double tmp;
	laserdata_cartesian tmpLaser;
	for(int i=0; i<segment.numberOfMeasures; i++)
	{
		tmpLaser = segment.measures.at(i);
		tmp = sqrt(tmpLaser.x * tmpLaser.x + tmpLaser.y * tmpLaser.y);
		if(tmp < leastMeasure)
		{
			leastLaser = tmpLaser;
			leastMeasure = tmp;
		}
	}
	relevantMeasures.push_back(leastLaser);
	//right border is always the last measure in a segment
	relevantMeasures.push_back(segment.measures.at(segment.numberOfMeasures-1));

	return relevantMeasures;
}
