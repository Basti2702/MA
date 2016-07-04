/*
 * IntervallMap.cpp
 *
 *  Created on: 27.06.2016
 *      Author: Sebastian Reinhart
 */

#include "IntervalMap.h"

IntervalMap::IntervalMap() {
	intervalLength = INTERVALL_LENGTH;
	numberOfIntervals = NUMBER_OF_INTERVALS;
	xSubInterval = 0;
	allocateIntervalMap();
}

IntervalMap::IntervalMap(int numberOfIntervals, int length)
{
	intervalLength = length;
	this->numberOfIntervals = numberOfIntervals;
	xSubInterval = 0;
	allocateIntervalMap();
}

IntervalMap::~IntervalMap() {
	freeIntervalMap();
}

/*
 * Build up binary tree containing all 100 Intervals
 */
void IntervalMap::allocateIntervalMap() {
/*	this->map = new std::set<PC>*[numberOfIntervalls];
	for (int i = 0; i < numberOfIntervalls; i++) {
		map[i] = new std::set<PC>();
	}*/
	map = new node;
	map->key_value = 49.5;
	map->tracks = NULL;
	map->left = NULL;
	map->right = NULL;

/*	//building up left tree
	insertInterval(map, 24.5);
	insertInterval(map, 12.5);
	insertInterval(map, 5.5);
	insertInterval(map, 4.5);
	insertInterval(map, 2.5);
	insertInterval(map, 3.5);
	insertInterval(map, 1.5);
	insertInterval(map, 0.5);
	insertInterval(map, 9.5);
	insertInterval(map, 7.5);
	insertInterval(map, 6.5);
	insertInterval(map, 8.5);
	insertInterval(map, 11.5);
	insertInterval(map, 10.5);
	insertInterval(map, 18.5);
	insertInterval(map, 15.5);
	insertInterval(map, 14.5);
	insertInterval(map, 17.5);
	insertInterval(map, 13.5);
	insertInterval(map, 16.5);
	insertInterval(map, 21.5);
	insertInterval(map, 20.5);
	insertInterval(map, 23.5);
	insertInterval(map, 22.5);
	insertInterval(map, 19.5);

	insertInterval(map, 12.5 + 25.0);
	insertInterval(map, 5.5 + 25.0);
	insertInterval(map, 4.5 + 25.0);
	insertInterval(map, 2.5 + 25.0);
	insertInterval(map, 3.5 + 25.0);
	insertInterval(map, 1.5 + 25.0);
	insertInterval(map, 0.5 + 25.0);
	insertInterval(map, 9.5 + 25.0);
	insertInterval(map, 7.5 + 25.0);
	insertInterval(map, 6.5 + 25.0 );
	insertInterval(map, 8.5 + 25.0);
	insertInterval(map, 11.5 + 25.0);
	insertInterval(map, 10.5 + 25.0);
	insertInterval(map, 18.5 + 25.0);
	insertInterval(map, 15.5 + 25.0);
	insertInterval(map, 14.5 + 25.0);
	insertInterval(map, 17.5 + 25.0);
	insertInterval(map, 13.5 + 25.0);
	insertInterval(map, 16.5 + 25.0);
	insertInterval(map, 21.5 + 25.0);
	insertInterval(map, 20.5 + 25.0);
	insertInterval(map, 23.5 + 25.0);
	insertInterval(map, 22.5 + 25.0);
	insertInterval(map, 19.5 + 25.0);

	//build right tree
	insertInterval(map, 24.5 + 50.0);
	insertInterval(map, 12.5 + 50.0);
	insertInterval(map, 5.5 + 50.0);
	insertInterval(map, 4.5 + 50.0);
	insertInterval(map, 2.5 + 50.0);
	insertInterval(map, 3.5 + 50.0);
	insertInterval(map, 1.5 + 50.0);
	insertInterval(map, 0.5 + 50.0);
	insertInterval(map, 9.5 + 50.0);
	insertInterval(map, 7.5 + 50.0);
	insertInterval(map, 6.5 + 50.0);
	insertInterval(map, 8.5 + 50.0);
	insertInterval(map, 11.5 + 50.0);
	insertInterval(map, 10.5 + 50.0);
	insertInterval(map, 18.5 + 50.0);
	insertInterval(map, 15.5 + 50.0);
	insertInterval(map, 14.5 + 50.0);
	insertInterval(map, 17.5 + 50.0);
	insertInterval(map, 13.5 + 50.0);
	insertInterval(map, 16.5 + 50.0);
	insertInterval(map, 21.5 + 50.0);
	insertInterval(map, 20.5 + 50.0);
	insertInterval(map, 23.5 + 50.0);
	insertInterval(map, 22.5 + 50.0);
	insertInterval(map, 19.5 + 50.0);

	insertInterval(map, 12.5 + 75.0);
	insertInterval(map, 5.5 + 75.0);
	insertInterval(map, 4.5 + 75.0);
	insertInterval(map, 2.5 + 75.0);
	insertInterval(map, 3.5 + 75.0);
	insertInterval(map, 1.5 + 75.0);
	insertInterval(map, 0.5 + 75.0);
	insertInterval(map, 9.5 + 75.0);
	insertInterval(map, 7.5 + 75.0);
	insertInterval(map, 6.5 + 75.0 );
	insertInterval(map, 8.5 + 75.0);
	insertInterval(map, 11.5 + 75.0);
	insertInterval(map, 10.5 + 75.0);
	insertInterval(map, 18.5 + 75.0);
	insertInterval(map, 15.5 + 75.0);
	insertInterval(map, 14.5 + 75.0);
	insertInterval(map, 17.5 + 75.0);
	insertInterval(map, 13.5 + 75.0);
	insertInterval(map, 16.5 + 75.0);
	insertInterval(map, 21.5 + 75.0);
	insertInterval(map, 20.5 + 75.0);
	insertInterval(map, 23.5 + 75.0);
	insertInterval(map, 22.5 + 75.0);
	insertInterval(map, 19.5 + 75.0);*/

	for(double i=0; i<4.0; i+=1.0)
	{
		insertInterval(map, 24.5 + i*25.0);
		insertInterval(map, 12.5 + i*25.0);
		insertInterval(map, 5.5 + i*25.0);
		insertInterval(map, 4.5 + i*25.0);
		insertInterval(map, 2.5 + i*25.0);
		insertInterval(map, 3.5 + i*25.0);
		insertInterval(map, 1.5 + i*25.0);
		insertInterval(map, 0.5 + i*25.0);
		insertInterval(map, 9.5 + i*25.0);
		insertInterval(map, 7.5 + i*25.0);
		insertInterval(map, 6.5 + i*25.0);
		insertInterval(map, 8.5 + i*25.0);
		insertInterval(map, 11.5 + i*25.0);
		insertInterval(map, 10.5 + i*25.0);
		insertInterval(map, 18.5 + i*25.0);
		insertInterval(map, 15.5 + i*25.0);
		insertInterval(map, 14.5 + i*25.0);
		insertInterval(map, 17.5 + i*25.0);
		insertInterval(map, 13.5 + i*25.0);
		insertInterval(map, 16.5 + i*25.0);
		insertInterval(map, 21.5 + i*25.0);
		insertInterval(map, 20.5 + i*25.0);
		insertInterval(map, 23.5 + i*25.0);
		insertInterval(map, 22.5 + i*25.0);
		insertInterval(map, 19.5 + i*25.0);
	}

	for(double i=0; i<100.0; i+=1.0)
	{
		insertInterval(map, i);
	}

}

void IntervalMap::freeIntervalMap() {
	/*for (int i = 0; i < numberOfIntervalls; i++) {
		delete map[i];
	}
	delete[] this->map;*/
	destroy_tree(map);
}

/**
 * 7|				|
 * 6|				|
 * 5|				|
 * 4|				|
 * 3|	   ||	    |
 * 2|				|
 * 1|				|
 * 0|				|
 *
 *
 * shifts the whole structure by the number of intervals that are covered within @param xMotion and the already
 * stored x-position in interval to the bottom to compensate ego motion
 */
void IntervalMap::shiftStructure(double xMotion)
{
	xSubInterval += xMotion;
	int numIntervals = (int)(xSubInterval - intervalLength);
	xSubInterval -= numIntervals;
	for (int i = 0; i < numIntervals; i++) {
		//delete everything contained in
		node* tmpNode = getInterval(map,0);
		std::list<PC>* tmp = tmpNode->tracks;
		tmp->clear();
		for (int j = 0; j < numberOfIntervals-1; j++)
		{
			getInterval(map,j)->tracks = getInterval(map,j + 1)->tracks;
		}
	//	map[0] = tmp;
		getInterval(map,numberOfIntervals-1)->tracks = tmp;
	}

}
/*
 * rotates the whole map by @param angle and shifts the PCs by @param yMotion
 */
void IntervalMap::rotateStructure(double angle, double yMotion) {
/*	std::map<int,std::vector<std::set<PC>::iterator> >ups;
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
	}*/
}

/**
 * Inserts new node with given key in the tree
 * NOTE: only the leafs of this tree contains the data we need, the rest is just for infrastructure
 */
void IntervalMap::insertInterval(node *leaf, double key)
{
	if(key < leaf->key_value)
	{
		if(leaf->left == NULL)
		{
			leaf->left = new node;
			leaf->left->key_value = key;
			leaf->left->left = NULL;
			leaf->left->right = NULL;
			if(fmod(key, 1.0) == 0.0)
			{
				leaf->left->tracks = new std::list<PC>();
			}
			else
			{
				leaf->left->tracks = NULL;
			}
		}
		else
		{
			insertInterval(leaf->left, key);
		}
	}
	else
	{
		if(leaf->right == NULL)
		{
			leaf->right= new node;
			leaf->right->key_value = key;
			leaf->right->left = NULL;
			leaf->right->right = NULL;
			if(fmod(key, 1.0) == 0.0)
			{
				leaf->right->tracks = new std::list<PC>();
			}
			else
			{
				leaf->right->tracks = NULL;
			}
		}
		else
		{
			insertInterval(leaf->right, key);
		}
	}
}

/**
 * frees the hole tree structure recursively
 */
void IntervalMap::destroy_tree(node *leaf)
{
  if(leaf!=NULL)
  {
    destroy_tree(leaf->left);
    destroy_tree(leaf->right);
    if(leaf->tracks != NULL)
    {
    	delete leaf->tracks;
    }
    delete leaf;
  }
}

/*
 * Returns the array of point cells for interval @param interval or NULL if an error occurs
 */
node* IntervalMap::getInterval(node* leaf, double interval)
{
	if(fmod(interval, 1.0) != 0 || interval < 0 || interval > 99)
	{
		//should never happen
		std::cout << "Interval out of bounds!!" << std::endl;
		return NULL;
	}

	//recursively get to the leafs and so the data we are looking for
	if(interval == leaf->key_value)
	{
		return leaf;
	}
	else if(interval < leaf->key_value)
	{
		return getInterval(leaf->left, interval);
	}
	else
	{
		return getInterval(leaf->right, interval);
	}
}
