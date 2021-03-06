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
#ifdef CREATE_MEASURES
	PGMReader pgmReader;
	double speed = 4.0/3.0;
	for(int i=0; i<NUM_MEASUREMENT; i++)
	{
		std::string number = getNextMeasureAsString(i);
		pgmReader.simulateLaserRays(number);

	/*	std::ofstream EMLMeasureFile;
		std::ostringstream measurePath;
		measurePath << "./Laserdata/EML" << number << ".txt";
		EMLMeasureFile.open (measurePath.str().c_str());
		EMLMeasureFile << ((double)i)*speed << " 0 0 120 0" << std::endl;
		EMLMeasureFile.close();*/
	}
#endif

	ConvoyTracker tracker;
	auto start = std::chrono::steady_clock::now();
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

		tracker.shiftConvoyHistory(deltaX);
		tracker.rotateConvoyHistory(deltaYaw, deltaY);

		//2. Predict current vehicle states
		std::vector<pcNode*> toDelete;
		for(int j = 99; j>=0; j--)
		{
			//TODO: move predicted vehicles in right interval and right y s
			int k = 0;
			pcNode* tracks = tracker.intervalMap.getPCfromInterval(j,k++);
			while(tracks != NULL)
			{
				//TODO: FIX BUG FOR STANDING CAR (negative x)!!!
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
				//		tracks->vehicle.subInvtl += tracks->vehicle.getX() - (j-CARINTERVAL);
						tracks->vehicle.setX((j-CARINTERVAL) + 0.5);
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
			//					tracks->vehicle.subInvtl += tracks->vehicle.getX() - (j-CARINTERVAL);
								tracks->vehicle.setX((j-CARINTERVAL) + 0.5);
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

	}
	auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start);
	std::cout << "Duration of ConvoyTracking: " << duration.count() << std::endl;
	tracker.visualizeConvoys();
	tracker.visualizeHistory();
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
    	Pos curPos;
    	curPos.x = x;
    	curPos.y = y;
    	curPos.theta = yaw;
    	EML.push_back(curPos);
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

			//only accept current vehicle as possible match if the majority of the distance is caused by the change of the x value
			//that´s because cars normally are moving more in x direction instead of y
			if(dist < minDist) //&& fabs(y-y1) < ASSOCIATION_Y_THRESHOLD)
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
			int intvl = floor(update->vehicle.getX());
			intvl += CARINTERVAL;
			update->vehicle.update(vehicles.at(i).stateVector);
			update->y = update->vehicle.getY();
			PointCell pc = update->vehicle;
			bool inserted = false;
			//TODO: move to correct interval if necessary
			if(update->vehicle.getX() > intvl+1-CARINTERVAL || update->vehicle.getX() < intvl-CARINTERVAL)
			{
				//vehicle has to be moved
				pcNode* newPC = intervalMap.insertNewTrack(update->vehicle);
				intervalMap.remove(update, intvl);
				inserted = true;

				if(newPC)
				{
					findConvoy(newPC->vehicle);
					history.at(newPC->vehicle.getID()).push_back(newPC->vehicle);
				}
			}
			else
			{
			//	update->vehicle.subInvtl += update->vehicle.getX() - (intvl-CARINTERVAL);
				update->vehicle.setX((intvl-CARINTERVAL) + 0.5);

				findConvoy(update->vehicle);
				history.at(update->vehicle.getID()).push_back(update->vehicle);
			}

			if(!inserted)
			{
				if(intervalMap.inorderTracks(intvl) == 1)
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
						findConvoy(update->vehicle);
						history.at(update->vehicle.getID()).push_back(update->vehicle);
					}
					if(!rightPos)
					{
						//node is wrong position -> delete PC from tree and add it again
						intervalMap.remove(update, intvl);
						pcNode* newpc = intervalMap.insertNewTrack(pc);
						if(newpc)
						{
							findConvoy(newpc->vehicle);
							history.at(newpc->vehicle.getID()).push_back(newpc->vehicle);
						}
					}

				}
			}
			trackedVehicles.at(minIndex) = tmp;
			trackedVehicles.pop_back();

			//update history of given vehicle


			std::cout << "Updated vehicle with ID " << pc.getID() << std::endl;

			findConvoy(pc);
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
	double x = vehicle.getX();
	int interval = floor(x);
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
				if(pc.getY() - 1.0 <= vehicle.getY() && vehicle.getY() <= pc.getY() + 1.0)
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
							EMLPos newPos;
							newPos.x = interval+0.5;
							newPos.y = vehicle.getY();
							newPos.theta = vehicle.getTheta();
							newPos.subIntvl = 0.5;
							convoys.at(j).tracks.push_back(newPos);
							convoyFound = true;
							break;
						}
						else if (it1 != currentConvoy.participatingVehicles.end())
						{
							EMLPos newPos;
							newPos.x = interval+0.5;
							newPos.y = vehicle.getY();
							newPos.theta = vehicle.getTheta();
							newPos.subIntvl = 0.5;
							convoys.at(j).tracks.push_back(newPos);
							currentConvoy.participatingVehicles.push_back(vehicle.getID());
							convoyFound = true;
							break;
						}
						else if (it2 != currentConvoy.participatingVehicles.end())
						{
							EMLPos newPos;
							newPos.x = interval+0.5;
							newPos.y = vehicle.getY();
							newPos.theta = vehicle.getTheta();
							newPos.subIntvl = 0.5;
							convoys.at(j).tracks.push_back(newPos);
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
						EMLPos newPos;
						newPos.x = interval+0.5;
						newPos.y = vehicle.getY();
						newPos.theta = vehicle.getTheta();
						newPos.subIntvl = 0.5;
						newConvoy.tracks.push_back(newPos);
						convoys.push_back(newConvoy);
					}
					return;
				}
			}
		}
	}
}

/**
 * shifts values of stored convoy tracks and history infos by given amount because it´s values are relative to own vehicle position
 */
void ConvoyTracker::shiftConvoyHistory(double x)
{
	std::vector<int> toDelete;

	//update history
	for (std::map<int,std::vector<PointCell> >::iterator it=history.begin(); it!=history.end(); ++it)
	{
		for(uint i = 0; i < it->second.size(); i++)
		{
			it->second.at(i).subInvtl += x;
			int numIntervals = (int) ((it->second.at(i).subInvtl) / INTERVALL_LENGTH);
			it->second.at(i).setX(it->second.at(i).getX() - numIntervals);
			it->second.at(i).subInvtl -= numIntervals;
		}

		//check whether current History is already behind our car
		if(it->second.at(it->second.size()-1).getX() < -5)
		{
			//if yes, mark history to delete
			toDelete.push_back(it->first);
		}
	}

	for(uint i=0; i<toDelete.size(); i++)
	{
		history.erase(toDelete.at(i));
	}

	toDelete.clear();

	//update Convoys
	for(uint i = 0; i < convoys.size(); i++)
	{
		for(uint j = 0; j < convoys.at(i).tracks.size(); j++)
		{
			convoys.at(i).tracks.at(j).subIntvl += x;
			int numIntervals = (int) ((convoys.at(i).tracks.at(j).subIntvl) / INTERVALL_LENGTH);
			convoys.at(i).tracks.at(j).x -= numIntervals;
			convoys.at(i).tracks.at(j).subIntvl -= numIntervals;
		}

		if(convoys.at(i).tracks.at(convoys.at(i).tracks.size()-1).x < -5)
		{
			toDelete.push_back(i);
		}
	}

	for(uint i=0; i<toDelete.size(); i++)
	{
		Convoy tmp = convoys.at(convoys.size()-1);
		convoys.at(toDelete.at(i)) = tmp;
		convoys.pop_back();
	}
}

/**
 * rotates and updates lateral position of stored convoy tracks and history infos by given amount because it´s values are relative to own vehicle position
 */
void ConvoyTracker::rotateConvoyHistory(double theta, double y)
{
	double angleInRadians = theta*M_PI/180.0;
	double mat[2][2] = { { cos(angleInRadians), -sin(angleInRadians) },
			{ sin(angleInRadians), cos(angleInRadians) } };
	//update history
	for (std::map<int,std::vector<PointCell> >::iterator it=history.begin(); it!=history.end(); ++it)
	{
		for(uint i = 0; i < it->second.size(); i++)
		{
			it->second.at(i).setY(it->second.at(i).getY() - y);
			it->second.at(i).setTheta(it->second.at(i).getTheta() - angleInRadians);

			double xAbs = it->second.at(i).getX();
			double yAbs = it->second.at(i).getY();

			xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
			yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

			it->second.at(i).setY(it->second.at(i).getY() - yAbs);
			it->second.at(i).subInvtl -= xAbs;
		}
	}

	//update Convoys
	for(uint i = 0; i < convoys.size(); i++)
	{
		for(uint j = 0; j < convoys.at(i).tracks.size(); j++)
		{
			convoys.at(i).tracks.at(j).y -= y;
			convoys.at(i).tracks.at(j).theta -= angleInRadians;

			double xAbs = convoys.at(i).tracks.at(j).x;
			double yAbs = convoys.at(i).tracks.at(j).y;

			xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
			yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

			convoys.at(i).tracks.at(j).y -= yAbs;
			convoys.at(i).tracks.at(j).subIntvl -= xAbs;
		}
	}
}

void ConvoyTracker::visualizeConvoys()
{
	visualizer.visualizeConvoys(EML, convoys);
}

void ConvoyTracker::visualizeHistory()
{
	visualizer.visualizeHistory(EML, history);
}

bool ConvoyTracker::checkConvoyForDuplicate(double x, Convoy c)
{
	for(uint i=0; i<c.tracks.size(); i++)
	{
		if(c.tracks.at(i).x == x)
		{
			return false;
		}
	}
	return true;
}
