/*
 * ConvoyTracker.cpp
 *
 *  Created on: 06.06.2016
 *      Author: basti
 */

#include "ConvoyTracker.h"


ConvoyTracker::ConvoyTracker() {
	// TODO Auto-generated constructor stub
	currentSpeed = 0;
	currentYawRate = 0;
	x = 0;
	y = 0;
	yaw = 0;
	xOld = 0;
	yOld = 0;
	yawOld = 0;
	ID = 0;
	convoyID = 0;
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
/*	IntervalMap test;
	PointCell pc;
	pc.stateVector.put(0,0, 10);
	pc.stateVector.put(1,0, 0);
	pc.stateVector.put(2,0, 45*M_PI / 180.0);
	pc.stateVector.put(3,0, 33.33);
	pc.stateVector.put(4,0, 0);
	pc.predict();
	std::cout << "X: " << pc.stateVector.get(0,0) << " Y: " << pc.stateVector.get(1,0) << " Theta: " << pc.stateVector.get(2,0) << " Vel: " << pc.stateVector.get(3,0) << " Phi: " << pc.stateVector.get(4,0) << std::endl;
	PointCell pc2;
	pc2.stateVector.put(0,0, 10.5);
	pc2.stateVector.put(1,0, 1.2);
	pc2.stateVector.put(2,0, 48*M_PI / 180.0);
	pc2.stateVector.put(3,0, 33.33);
	pc2.stateVector.put(4,0, 0);
	pc.update(pc2.stateVector);
	std::cout << "NewState: X: " << pc2.stateVector.get(0,0) << " Y: " << pc2.stateVector.get(1,0) << " Theta: " << pc2.stateVector.get(2,0) << " Vel: " << pc2.stateVector.get(3,0) << " Phi: " << pc2.stateVector.get(4,0) << std::endl;
	std::cout << "StateVec after Update X: " << pc.stateVector.get(0,0) << " Y: " << pc.stateVector.get(1,0) << " Theta: " << pc.stateVector.get(2,0) << " Vel: " << pc.stateVector.get(3,0) << " Phi: " << pc.stateVector.get(4,0) << std::endl;

	test.insertNewTrack(pc);
	test.insertNewTrack(pc2);
	pcNode* testpc = test.getPCfromInterval(10, 1);*/

	std::vector<PointCell> vehicles;
	for(int i=0; i<NUM_MEASUREMENT; i++)
	{
		std::vector<pcNode*> trackedVehicles;
		std::string number = getNextMeasureAsString(i);
		tracker.readEMLData(number);
		vehicles = tracker.reader.processLaserData(number,tracker.getCurrentSpeed(), tracker.getCurrentYawRate());

		//1. Compensate own vehicle motion
		double deltaX = tracker.getX() - tracker.getXOld();
		double deltaY = tracker.getY() - tracker.getYOld();
		double deltaYaw = tracker.getYaw() - tracker.getYawOld();

		tracker.intervalMap.shiftStructure(deltaX);
		tracker.intervalMap.rotateStructure(deltaYaw, deltaY);

		//2. Predict current vehicle states
		std::vector<pcNode*> toDelete;
		for(int j = 99; j>=0; j--)
		{
			//TODO: move predicted vehicles in right interval and right y s
			int k = 0;
			pcNode* tracks = tracker.intervalMap.getPCfromInterval(j,k++);
			while(tracks != NULL)
			{
				tracks->vehicle.predict();
				tracks->y = tracks->vehicle.getY();
				PointCell pc = tracks->vehicle;
				//tracker.intervalMap.deletePCfromInterval(j, pc);
				pcNode* tmp = NULL;
				if(tracks->vehicle.getX() > j+1-CARINTERVAL)
				{
					//vehicle has to be moved
					tmp = tracker.intervalMap.insertNewTrack(tracks->vehicle);
					toDelete.push_back(tracks);
				}
				if(tmp != NULL)
				{
					trackedVehicles.push_back(tmp);
				}
				else
				{
					//update current node
					if(tracker.intervalMap.inorderTracks(j) == 1)
					{
						trackedVehicles.push_back(tracks);
					}
					else
					{
						bool rightPos = false;
						if((tracks->left != NULL && tracks->left->y < tracks->y) || tracks->left == NULL)
						{
							//left ok
							//now check right child
							if((tracks->right != NULL && tracks->right->y > tracks->y) || tracks->right == NULL)
							{
								//right ok -> node still in right position
								rightPos = true;
								trackedVehicles.push_back(tracks);
							}
						}
						if(!rightPos)
						{
							//node is wrong position -> delete PC from tree and add it again
							toDelete.push_back(tracks);
							tmp = tracker.intervalMap.insertNewTrack(pc);
							if(tmp != NULL)
							{
								trackedVehicles.push_back(tmp);
							}
						}

					}
				}
				tracks = tracker.intervalMap.getPCfromInterval(j,k++);
			}
			for(uint m = 0; m < toDelete.size(); m++)
			{
				tracker.intervalMap.remove(toDelete.at(m), j);
			}
			toDelete.clear();
		}
		//3. Associate and Update
		tracker.associateAndUpdate(vehicles, trackedVehicles);

		//4. find/update Convoytracks

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
			switch (dataCnt)
			{
				case 1:
				{
					//x in m
					xOld = x;
					x = atof(segment.c_str());
					++dataCnt;
					break;
				}
				case 2:
				{
					//y in m
					yOld = y;
					y = atof(segment.c_str());
					++dataCnt;
					break;
				}
				case 3:
				{
					//yaw in °
					yawOld = yaw;
					yaw = atof(segment.c_str());
					++dataCnt;
					break;
				}
				case 4:
				{
					//velocity in kmh
					//Compute value in m/s
					currentSpeed = atof(segment.c_str()) / 3.6;
					++dataCnt;
					break;
				}
				case 5:
				{
					//yaw rate in °/s
					//Compute value in rad/s
					currentYawRate = atof(segment.c_str()) * M_PI / 180.0;
					break;
				}
			}
    	}
    }
}

double ConvoyTracker::getCurrentSpeed() const {
	return currentSpeed;
}

void ConvoyTracker::setCurrentSpeed(double currentSpeed) {
	this->currentSpeed = currentSpeed;
}

double ConvoyTracker::getCurrentYawRate() const {
	return currentYawRate;
}

void ConvoyTracker::setCurrentYawRate(double currentYawRate) {
	this->currentYawRate = currentYawRate;
}

double ConvoyTracker::getX() const {
	return x;
}

void ConvoyTracker::setX(double x) {
	this->x = x;
}

double ConvoyTracker::getXOld() const {
	return xOld;
}

void ConvoyTracker::setXOld(double old) {
	xOld = old;
}

double ConvoyTracker::getY() const {
	return y;
}

void ConvoyTracker::setY(double y) {
	this->y = y;
}

double ConvoyTracker::getYaw() const {
	return yaw;
}

void ConvoyTracker::setYaw(double yaw) {
	this->yaw = yaw;
}

double ConvoyTracker::getYawOld() const {
	return yawOld;
}

void ConvoyTracker::setYawOld(double yawOld) {
	this->yawOld = yawOld;
}

double ConvoyTracker::getYOld() const {
	return yOld;
}

void ConvoyTracker::setYOld(double old) {
	yOld = old;
}

/**
 * Searches for corresponding vehicles using Global Nearest Neighbor algorithm and updates the results
 */
void ConvoyTracker::associateAndUpdate(std::vector<PointCell> vehicles, std::vector<pcNode*> trackedVehicles)
{
	for(uint i = 0; i<vehicles.size(); i++)
	{
		double x = vehicles.at(i).getX();
		double y = vehicles.at(i).getY();
		double theta = vehicles.at(i).getTheta();

		double minDist = INT_MAX;
		double minIndex = INT_MAX;

		std::cout << "X: " << x << " Y: " << y << " Theta: " << theta <<std::endl;
		for(uint j = 0; j<trackedVehicles.size(); j++)
		{
			double x1 = trackedVehicles.at(j)->vehicle.getX();
			double y1 = trackedVehicles.at(j)->vehicle.getY();
			double theta1 = trackedVehicles.at(j)->vehicle.getTheta();

			std::cout << "X1: " << x1 << " Y1: " << y1<< " Theta1: " << theta1 <<std::endl;

			double dist = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1) + (theta - theta1)*(theta - theta1));
			if(dist < minDist)
			{
				minDist = dist;
				minIndex = j;
			}
		}
		std::cout << "Min distance: " << minDist << std::endl;
		if(minDist > ASSOCIATION_THRESHOLD)
		{
			//do not associate vehicles with to big distance in between
			//create new track instead
			++ID;
			vehicles.at(i).setID(ID);
			intervalMap.insertNewTrack(vehicles.at(i));
			std::vector<PointCell> newHist;
			newHist.push_back(vehicles.at(i));
			history.insert(std::pair<int, std::vector<PointCell> > (ID, newHist));
			std::cout << "Added new Vehicle with ID " << ID << std::endl;

//			findConvoy(vehicles.at(i));
		}
		else
		{
			pcNode* tmp = trackedVehicles.at(trackedVehicles.size() -1 );
			pcNode* update = trackedVehicles.at(minIndex);
			int intvl = (int) update->vehicle.getX();
			intvl += CARINTERVAL;
			update->vehicle.update(vehicles.at(i).stateVector);
			update->y = update->vehicle.getY();
			PointCell pc = update->vehicle;
			//TODO: move to correct interval if necessary
			if(update->vehicle.getX() > intvl+1-CARINTERVAL || update->vehicle.getX() < intvl-CARINTERVAL)
			{
				//vehicle has to be moved
				intervalMap.insertNewTrack(update->vehicle);
				intervalMap.remove(update, intvl);
			}


			else if(intervalMap.inorderTracks(intvl) == 1)
			{
			}
			else
			{
				bool rightPos = false;
				if((update->left != NULL && update->left->y < update->y) || update->left == NULL)
				{
					//left ok
					//now check right child
					if((update->right != NULL && update->right->y > update->y) || update->right == NULL)
					{
						//right ok -> node still in right position
						rightPos = true;
					}
				}
				if(!rightPos)
				{
					//node is wrong position -> delete PC from tree and add it again
					intervalMap.remove(update, intvl);
					intervalMap.insertNewTrack(pc);
				}

			}

			trackedVehicles.at(minIndex) = tmp;
			trackedVehicles.pop_back();

			//update history of given vehicle
			history.at(pc.getID()).push_back(pc);

			std::cout << "Updated vehicle with ID " << pc.getID() << std::endl;

//			findConvoy(pc);
		}
	}

	for(uint k = 0; k < trackedVehicles.size(); k++)
	{
		//delete all tracks that could not be matched
		intervalMap.removeVehicle(trackedVehicles.at(k));
	}
}

void ConvoyTracker::findConvoy(PointCell vehicle)
{
	for (std::map<int,std::vector<PointCell> >::iterator it=history.begin(); it!=history.end(); ++it)
	{
		if(it->first == vehicle.getID())
		{
			continue;
		}

		for(uint i = 0; i < it->second.size(); i++)
		{
			PointCell pc = it->second.at(i);
			if(pc.getX() - 0.5 <= vehicle.getX() && vehicle.getX() <= pc.getX() + 0.5)
			{
				if(pc.getY() - 0.5 <= vehicle.getY() && vehicle.getY() <= pc.getY() + 0.5)
				{
					//current vehicle position matches with one from history -> add this pointcell to convoy
					int id1 = pc.getID();
					int id2 = vehicle.getID();
					bool convoyFound = false;
					for(uint j = 0; j < convoys.size(); j++)
					{
						std::vector<int>::iterator it1, it2;
						Convoy currentConvoy = convoys.at(j);
						it1 = std::find(currentConvoy.participatingVehicles.begin(), currentConvoy.participatingVehicles.end(), id1);
						it2 = std::find(currentConvoy.participatingVehicles.begin(), currentConvoy.participatingVehicles.end(), id2);
						if(it1 != currentConvoy.participatingVehicles.end() && it2 != currentConvoy.participatingVehicles.end())
						{
							//convoy already exists with both IDS
							intervalMap.insertPC(currentConvoy.track, vehicle);
							convoyFound = true;
							break;
						}
						else if (it1 != currentConvoy.participatingVehicles.end())
						{
							intervalMap.insertPC(currentConvoy.track, vehicle);
							currentConvoy.participatingVehicles.push_back(vehicle.getID());
							convoyFound = true;
							break;
						}
						else if (it2 != currentConvoy.participatingVehicles.end())
						{
							intervalMap.insertPC(currentConvoy.track, vehicle);
							currentConvoy.participatingVehicles.push_back(pc.getID());
							convoyFound = true;
							break;
						}
					}

					if(!convoyFound)
					{
						Convoy newConvoy;
						newConvoy.ID = convoyID++;
						newConvoy.participatingVehicles.push_back(pc.getID());
						newConvoy.participatingVehicles.push_back(vehicle.getID());
						intervalMap.insertPC(newConvoy.track, vehicle);
					}
					return;
				}
			}
		}
	}
}
