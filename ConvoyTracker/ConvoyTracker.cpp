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
	ConvoyTracker tracker;
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
		std::vector<pcNode*> trackedVehicles;
		std::string number = getNextMeasureAsString(i);
		tracker.readEMLData(number);
		vehicles = tracker.reader.processLaserData(number,tracker.currentSpeed, tracker.currentYawRate);

		//1. Compensate own vehicle motion
		tracker.intervalMap.shiftStructure(tracker.currentSpeed/TIMESTAMP);
		tracker.intervalMap.rotateStructure(tracker.currentYawRate/TIMESTAMP, 0);

		//2. Predict current vehicle states
		std::vector<pcNode*> toDelete;
		for(int j = 99; j>=0; j--)
		{
			//TODO: move predicted vehicles in right interval
			pcNode* tracks = tracker.intervalMap.getPCfromInterval(i,0);
			int k = 1;
			while(tracks != NULL)
			{
				trackedVehicles.push_back(tracks);
				tracks->vehicle.predict();
				if(tracks->vehicle.getX() > i+1-CARINTERVAL)
				{
					//vehicle has to be moved
					tracker.intervalMap.insertNewTrack(tracks->vehicle);
					toDelete.push_back(tracks);
				}
				tracks = tracker.intervalMap.getPCfromInterval(i,k++);
			}
			for(uint m = 0; m < toDelete.size(); m++)
			{
				tracker.intervalMap.remove(toDelete.at(m), j);
			}
			toDelete.clear();
		}
		//3. Associate and Update

	}
	return 0;
}

/**
 * Stores current Speed and yaw rate from file to class variables
 */
void ConvoyTracker::readEMLData(std::string number)
{
	std::ostringstream measurePath;
	measurePath << EMLPATH << number << ".txt";
	std::cout << measurePath.str() << std::endl;
    std::ifstream input(measurePath.str().c_str());
    std::string line;
    std::string segment;

    if(std::getline( input, line ))
    {
    	std::stringstream ss;

    	ss << line;

    	int dataCnt = 1;

    	while(std::getline(ss, segment, ' '))
    	{
    		if(dataCnt == 1)
    		{
    			//First value in file corresponds to current velocity in kmh
    			//Compute value in m/s
    			currentSpeed = atof(segment.c_str()) / 3.6;
    			++dataCnt;
    		}
    		else
    		{
    			//second value corresponds to yaw rate in °/s
    			//Compute value in rad/s
    			currentYawRate = atof(segment.c_str()) * M_PI / 180.0;
    			break;
    		}
    	}
    }
}

/**
 * Searches for corresponding vehicles using Global Nearest Neighbor algorithm and updates the results
 */
void ConvoyTracker::associateAndUpdate(std::vector<PointCell> vehicles, std::vector<pcNode*> trackedVehicles)
{
	for(uint i = 0; i<vehicles.size(); i++)
	{
		double x = vehicles.at(i).stateVector.get(0,0);
		double y = vehicles.at(i).stateVector.get(1,0);
		double theta = vehicles.at(i).stateVector.get(2,0);

		double minDist = INT_MAX;
		double minIndex = INT_MAX;

		for(uint j = 0; j<trackedVehicles.size(); j++)
		{
			double x1 = trackedVehicles.at(i)->vehicle.stateVector.get(0,0);
			double y1 = trackedVehicles.at(i)->vehicle.stateVector.get(1,0);
			double theta1 = trackedVehicles.at(i)->vehicle.stateVector.get(2,0);

			double dist = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1) + (theta - theta1)*(theta - theta1));
			if(dist < minDist)
			{
				minDist = dist;
				minIndex = j;
			}
		}

		if(minDist > ASSOCIATION_THRESHOLD)
		{
			//do not associate vehicles with to big distance in between
			//create new track instead
			intervalMap.insertNewTrack(vehicles.at(i));
		}
		else
		{
			pcNode* tmp = trackedVehicles.at(trackedVehicles.size() -1 );
			pcNode* update = trackedVehicles.at(minIndex);
			update->vehicle.update(vehicles.at(i).stateVector);
			trackedVehicles.at(minIndex) = tmp;
			trackedVehicles.pop_back();
		}
	}

	for(uint k = 0; k < trackedVehicles.size(); k++)
	{
		//delete all tracks that could not be matched

	}
}
