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
	map->key_value = 50;
	map->tracks = NULL;
	map->left = NULL;
	map->right = NULL;

	insertInterval(map, 25);
	insertInterval(map, 75);
	for(int i=0; i<4; i++)
	{
		insertInterval(map, 12 + i*25);
		insertInterval(map, 6 + i*25);
		insertInterval(map, 3 + i*25);
		insertInterval(map, 5 + i*25);
		insertInterval(map, 4 + i*25);
		insertInterval(map, 1 + i*25);
		insertInterval(map, 2 + i*25);
		insertInterval(map, 9 + i*25);
		insertInterval(map, 8 + i*25);
		insertInterval(map, 7 + i*25);
		insertInterval(map, 11 + i*25);
		insertInterval(map, 10 + i*25);
		insertInterval(map, 18 + i*25);
		insertInterval(map, 15 + i*25);
		insertInterval(map, 14 + i*25);
		insertInterval(map, 13 + i*25);
		insertInterval(map, 17 + i*25);
		insertInterval(map, 16 + i*25);
		insertInterval(map, 21 + i*25);
		insertInterval(map, 20 + i*25);
		insertInterval(map, 19 + i*25);
		insertInterval(map, 23 + i*25);
		insertInterval(map, 22 + i*25);
		insertInterval(map, 24 + i*25);
	}
	insertInterval(map, 0);
	//debug
//	inorder(map);

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
		pcNode* tmp = tmpNode->tracks;
		destroy_PCTree(tmp);
		for (int j = 0; j < numberOfIntervals-1; j++)
		{
			getInterval(map,j)->tracks = getInterval(map,j + 1)->tracks;
		}
	//	map[0] = tmp;
		getInterval(map,numberOfIntervals-1)->tracks = NULL;
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
			leaf->left->tracks = NULL;
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
			leaf->right->tracks = NULL;
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
    	destroy_PCTree(leaf->tracks);
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

void IntervalMap::inorder(node* tree)
{
	if(tree->left != NULL)
	{
			inorder(tree->left);
	}
	std::cout << tree->key_value << std::endl;
	if(tree->right != NULL)
	{
		inorder(tree->right);
	}
}

void IntervalMap::insertPCintoInterval(int interval, PC vehicle)
{
	node* intvl = getInterval(map, interval);
	if(intvl == NULL)
	{
		std::cerr << "COULD NOT FIND INTERVALL!!" << std::endl;
	}
	insertPC(intvl->tracks, NULL, vehicle);
}

void IntervalMap::insertPC(pcNode* leaf, pcNode* parent, PC vehicle)
{

	double key = vehicle.y;
	std::cout << "Key: " << key << std::endl;
	if(leaf != NULL)
	{
		if(key < leaf->y)
		{
			insertPC(leaf->left, leaf, vehicle);
		}
		else if(key > leaf->y)
		{
			insertPC(leaf->right, leaf, vehicle);
		}
		else
		{
			//should never happen
			std::cerr << "ERROR WHILE INSERTING NEW PC: VALUE ALREADY IN LIST!!"<< std::endl;
			return;
		}
	}
	else
	{
		std::cout << "created new node for key" << std::endl;
		leaf = new pcNode;
		leaf->y = key;
		leaf->vehicle = vehicle;
		leaf->left = NULL;
		leaf->right = NULL;
		leaf->parent = parent;
	}
}

void IntervalMap::inorderTracks(int interval)
{
	node* intvl = getInterval(map,interval);
	if(intvl == NULL)
	{
		std::cerr << "COULD NOT FIND INTERVALL!!" << std::endl;
	}
	inorder(intvl->tracks);
}

void IntervalMap::inorder(pcNode* root)
{
	std::cout << "piep" << std::endl;
	if(root->left != NULL)
	{
		std::cout << "piep" << std::endl;
			inorder(root->left);
	}
	std::cout << "get root" << std::endl;
	std::cout << root->y << std::endl;
	if(root->right != NULL)
	{
		inorder(root->right);
	}
}
void IntervalMap::deletePCfromInterval(int interval, PC vehicle)
{
	node* intvl = getInterval(map,interval);
	if(intvl->tracks == NULL)
	{
		std::cerr << "ERROR WHILE DELETING PC FROM INTERVAL: NO PC CONTAINED IN INTERVAL!" << std::endl;
		return;
	}

}

void IntervalMap::deletePC(pcNode* leaf, PC vehicle)
{
	double key = vehicle.y;
	if(leaf->y == key)
	{
		remove(leaf);
	}
	else if(key < leaf->y)
	{
		deletePC(leaf->left, vehicle);
	}
	else
	{
		deletePC(leaf->right, vehicle);
	}
}

void IntervalMap::destroy_PCTree(pcNode* root)
{
	  if(root!=NULL)
	  {
	    destroy_PCTree(root->left);
	    destroy_PCTree(root->right);
	    delete root;
	  }
}
