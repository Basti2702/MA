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

IntervalMap::IntervalMap(int numberOfIntervals, int length) {
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
	for (int i = 0; i < 4; i++) {
		insertInterval(map, 12 + i * 25);
		insertInterval(map, 6 + i * 25);
		insertInterval(map, 3 + i * 25);
		insertInterval(map, 5 + i * 25);
		insertInterval(map, 4 + i * 25);
		insertInterval(map, 1 + i * 25);
		insertInterval(map, 2 + i * 25);
		insertInterval(map, 9 + i * 25);
		insertInterval(map, 8 + i * 25);
		insertInterval(map, 7 + i * 25);
		insertInterval(map, 11 + i * 25);
		insertInterval(map, 10 + i * 25);
		insertInterval(map, 18 + i * 25);
		insertInterval(map, 15 + i * 25);
		insertInterval(map, 14 + i * 25);
		insertInterval(map, 13 + i * 25);
		insertInterval(map, 17 + i * 25);
		insertInterval(map, 16 + i * 25);
		insertInterval(map, 21 + i * 25);
		insertInterval(map, 20 + i * 25);
		insertInterval(map, 19 + i * 25);
		insertInterval(map, 23 + i * 25);
		insertInterval(map, 22 + i * 25);
		insertInterval(map, 24 + i * 25);
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
void IntervalMap::shiftStructure(double xMotion) {
	xSubInterval += xMotion;
	int numIntervals = (int) (xSubInterval / intervalLength);
	xSubInterval -= numIntervals;
	for (int i = 0; i < numIntervals; i++) {
		//delete everything contained in
		node* tmpNode = getInterval(map, 0);
		pcNode* tmp = tmpNode->tracks;
		destroy_PCTree(tmp);
		for (int j = 0; j < numberOfIntervals - 1; j++)
		{
			node* intvl = getInterval(map, j);
			intvl->tracks = getInterval(map, j + 1)->tracks;
			for(int k = 0; k<inorderPC(intvl->tracks,0); k++)
			{
				pcNode* pc = getPCfromInterval(j, k);
				pc->vehicle.setX(j-CARINTERVAL +0.5);
			}

		}
		//	map[0] = tmp;
		getInterval(map, numberOfIntervals - 1)->tracks = NULL;
	}

}
/*
 * rotates the whole map by @param angle and shifts the PCs by @param yMotion
 */
void IntervalMap::rotateStructure(double angle, double yMotion) {
	//map for temporary storage of PC that should be moved one interval up
	std::vector<pcNode*> prevUp;
	std::vector<pcNode*> curUp;
	double angleInRadians = angle * M_PI / 180;
	//1.Step correct directions of stored PCs
	for (int j = 0; j < numberOfIntervals; j++) {
		node* intvl = getInterval(map, j);
		for (int i = 0; i < inorderPC(intvl->tracks,0); i++)
		{
			PointCell vehicle;
			pcNode* pc = getPCfromInterval(j, i);
			//delete PC from tree , because key value changes
			//if PC stays within this interval, we will add it later again to ensure we still have an orderd tree
			vehicle = pc->vehicle;
			deletePC(intvl->tracks, pc->vehicle, j);
			//change values directly
			vehicle.setY(vehicle.getY() - yMotion);
			vehicle.setTheta(vehicle.getTheta() - angleInRadians);

			//2. compensate rotation
			double xAbs = (j - CARINTERVAL + 0.5) * intervalLength
					- xSubInterval;
			double yAbs = vehicle.stateVector.get(1,0);


			double mat[2][2] = { { cos(angleInRadians), -sin(angleInRadians) },
					{ sin(angleInRadians), cos(angleInRadians) } };
			xAbs = (mat[0][0] * xAbs + mat[0][1] * yAbs) - xAbs;
			yAbs = (mat[1][0] * xAbs + mat[1][1] * yAbs) - yAbs;

		//	vehicle.y -= yAbs;
			vehicle.setY(vehicle.getY() - yAbs);
			if (xAbs > 0.5 * intervalLength) {
				//move one intervall up
			//	vehicle.x += 1;
				vehicle.setX(vehicle.getX() + 1);
				curUp.push_back(pc);
			} else if (xAbs < -0.5 * intervalLength) {
				//move one intervall down;
				if(j > 0)
				{
			//		vehicle.x -= 1;
					vehicle.setX(vehicle.getX() - 1);
					insertPCintoInterval(j-1, vehicle);
				}
		//		deletePC(intvl->tracks, pc->vehicle, j);
			}
			else
			{
				//vehicle.x -= xAbs;
				//vehicle.setX(vehicle.getX() - xAbs);
				vehicle.subInvtl -= xAbs;
				//add vehicle to current interval again
				insertPCintoInterval(j, vehicle);
			}
		}
		for(uint i = 0; i < prevUp.size(); i++)
		{
			pcNode* tmp = prevUp.at(i);
			insertPCintoInterval(j, tmp->vehicle);
		}
		prevUp.clear();
		prevUp = curUp;
		uint s = curUp.size();
		curUp.clear();
		if(prevUp.size() != s)
		{
			std::cerr << "COPY NEEDED!!" << std::endl;
		}
	}
}

/**
 * Inserts new node with given key in the tree
 * NOTE: only the leafs of this tree contains the data we need, the rest is just for infrastructure
 */
void IntervalMap::insertInterval(node *leaf, double key) {
	if (key < leaf->key_value) {
		if (leaf->left == NULL) {
			leaf->left = new node;
			leaf->left->key_value = key;
			leaf->left->left = NULL;
			leaf->left->right = NULL;
			leaf->left->tracks = NULL;
		} else {
			insertInterval(leaf->left, key);
		}
	} else {
		if (leaf->right == NULL) {
			leaf->right = new node;
			leaf->right->key_value = key;
			leaf->right->left = NULL;
			leaf->right->right = NULL;
			leaf->right->tracks = NULL;
		} else {
			insertInterval(leaf->right, key);
		}
	}
}

/**
 * frees the hole tree structure recursively
 */
void IntervalMap::destroy_tree(node *leaf) {
	if (leaf != NULL) {
		destroy_tree(leaf->left);
		destroy_tree(leaf->right);
		if (leaf->tracks != NULL) {
			destroy_PCTree(leaf->tracks);
		}
		delete leaf;
	}
}

/*
 * Returns the array of point cells for interval @param interval or NULL if an error occurs
 */
node* IntervalMap::getInterval(node* leaf, double interval) {
	if (fmod(interval, 1.0) != 0 || interval < 0 || interval > 99) {
		//should never happen
		std::cout << "Interval out of bounds!!" << std::endl;
		return NULL;
	}

	//recursively get to the leafs and so the data we are looking for
	if (interval == leaf->key_value) {
		return leaf;
	} else if (interval < leaf->key_value) {
		return getInterval(leaf->left, interval);
	} else {
		return getInterval(leaf->right, interval);
	}
}

void IntervalMap::inorder(node* tree) {
	if (tree->left != NULL) {
		inorder(tree->left);
	}
	std::cout << tree->key_value << std::endl;
	if (tree->right != NULL) {
		inorder(tree->right);
	}
}

pcNode* IntervalMap::insertPCintoInterval(int interval, PointCell vehicle) {
	node* intvl = getInterval(map, interval);
	if (intvl == NULL) {
		std::cerr << "COULD NOT FIND INTERVALL!!" << std::endl;
	}
	if (intvl->tracks != NULL) {
		return insertPC(intvl->tracks, vehicle);
	} else {
		intvl->tracks = new pcNode;
		intvl->tracks->y = vehicle.stateVector.get(1,0);
		intvl->tracks->vehicle = vehicle;
		intvl->tracks->left = NULL;
		intvl->tracks->right = NULL;
		intvl->tracks->parent = NULL;
		return intvl->tracks;
	}
}

/**
 * inserts the PC @param vehicle into the treeNode @param leaf
 */
pcNode* IntervalMap::insertPC(pcNode* leaf, PointCell vehicle) {

	double key = vehicle.stateVector.get(1,0);
	if (key < leaf->y) {
		if (leaf->left != NULL) {
			return insertPC(leaf->left, vehicle);
		} else {
			leaf->left = new pcNode;
			leaf->left->y = key;
			leaf->left->vehicle = vehicle;
			leaf->left->left = NULL;
			leaf->left->right = NULL;
			leaf->left->parent = leaf;
			return leaf->left;
		}

	} else if (key > leaf->y) {
		if (leaf->right != NULL) {
			return insertPC(leaf->right, vehicle);
		} else {
			leaf->right = new pcNode;
			leaf->right->y = key;
			leaf->right->vehicle = vehicle;
			leaf->right->left = NULL;
			leaf->right->right = NULL;
			leaf->right->parent = leaf;
			return leaf->right;
		}
	} else {
		//should never happen
		std::cerr << "ERROR WHILE INSERTING NEW PC: VALUE ALREADY IN LIST!!"
				<< std::endl;
		return NULL;
	}
}

/**
 * Inorder traversal of tree in given interval
 * returns the number of elements in this tree
 */
int IntervalMap::inorderTracks(int interval) {
	node* intvl = getInterval(map, interval);
	if (intvl == NULL) {
		std::cerr << "COULD NOT FIND INTERVALL!!" << std::endl;
	}
	return inorderPC(intvl->tracks, 1);
}

int IntervalMap::inorderPC(pcNode* root, int print) {
	int count = 0;
	if (root == NULL) {
		return 0;
	}
	if (root->left != NULL) {
		count += inorderPC(root->left, print);
	}
	++count;
	if(print)
	{
		std::cout << root->y << " ";
	}
	if (root->right != NULL) {
		count += inorderPC(root->right, print);
	}
	return count;
}
void IntervalMap::deletePCfromInterval(int interval, PointCell vehicle) {
	node* intvl = getInterval(map, interval);
	if (intvl->tracks == NULL) {
		std::cerr
				<< "ERROR WHILE DELETING PC FROM INTERVAL: NO PC CONTAINED IN INTERVAL!"
				<< std::endl;
		return;
	} else {
		deletePC(intvl->tracks, vehicle, interval);
	}
}

void IntervalMap::deletePC(pcNode* leaf, PointCell vehicle, int interval) {
	double key = vehicle.stateVector.get(1,0);
	if (leaf->y == key) {
		remove(leaf, interval);
	} else if (key < leaf->y) {
		deletePC(leaf->left, vehicle, interval);
	} else {
		deletePC(leaf->right, vehicle, interval);
	}
}

void IntervalMap::destroy_PCTree(pcNode* root) {
	if (root != NULL) {
		destroy_PCTree(root->left);
		destroy_PCTree(root->right);
		delete root;
	}
}

pcNode** IntervalMap::get_parent_ptr(pcNode* _node, int interval) {
	if (_node->parent == 0) {
		node* intvl = getInterval(map, interval);
		return &intvl->tracks;
	} else if (is_left_child(_node)) {
		return &_node->parent->left;
	} else {
		return &_node->parent->right;
	}
}

//helper methods for removing elements from pcNode
void IntervalMap::swap_near_nodes(pcNode*child, pcNode*parent, int interval) {
	// Als erstes passen wir den unbeteiligten Großelternknoten an.
	*get_parent_ptr(parent, interval) = child;

	// Anschließend werden die Kind- und Elternzeiger ausgetauscht.
	std::swap(parent->left, child->left);
	std::swap(parent->right, child->right);
	std::swap(parent->parent, child->parent);

	// Da eines der Kinder getauscht wird benötigt es eine
	// sonder Behandlung.
	if (child->left == child)
		child->left = parent;
	else
		child->right = parent;

	// Nun sind alle Kindzeiger richtig und die Elternzeiger können
	// dem angepasst werden.
	if (child->left)
		child->left->parent = child;
	if (child->right)
		child->right->parent = child;
	if (parent->left)
		parent->left->parent = parent;
	if (parent->right)
		parent->right->parent = parent;

	// Na wer ist sich noch sicher ob wir nicht
	// bereits Zeigersalat haben? Besser testen!
}

void IntervalMap::swap_far_nodes(pcNode*a, pcNode*b, int interval) {
	// Zuerst updaten wir die Zeiger der Eltern
	*get_parent_ptr(a, interval) = b;
	*get_parent_ptr(b, interval) = a;

	// Danach der Kinder
	if (a->left)
		a->left->parent = b;
	if (a->right)
		a->right->parent = b;
	if (b->left)
		b->left->parent = a;
	if (b->right)
		b->right->parent = a;

	// Und als letztes die der beiden Knoten
	std::swap(a->left, b->left);
	std::swap(a->right, b->right);
	std::swap(a->parent, b->parent);
}

void IntervalMap::swap_nodes(pcNode*a, pcNode*b, int interval) {
	if (a->parent == b) {
		std::cout << "swap near a b" << std::endl;
		swap_near_nodes(a, b, interval);
	} else if (b->parent == a) {
		std::cout << "swap near b a" << std::endl;
		swap_near_nodes(b, a, interval);
	} else {
		std::cout << "swap far" << std::endl;
		swap_far_nodes(a, b, interval);
	}
}

bool IntervalMap::is_left_child(const pcNode*node) {
	if (node->parent == 0)
		return false; // Die Wurzel ist kein Kind
	else
		return node->parent->left == node;
}

bool IntervalMap::is_right_child(const pcNode*node) {
	if (node->parent == 0)
		return false; // Die Wurzel ist kein Kind
	else
		return node->parent->right == node;
}

pcNode* IntervalMap::get_max(pcNode*node) {
	pcNode*now = node;
	while (now->right)
		now = now->right;
	return now;
}

pcNode* IntervalMap::get_prev_node(pcNode*now) {
	if (now->left)
		return get_max(now->left);
	else {
		while (now) {
			if (is_right_child(now))
				return now->parent;
			else
				now = now->parent;
		}
		return 0; // Wir sind am Anfang angekommen
	}
}

void IntervalMap::remove(pcNode*node, int interval) {
	// Ein Blatt
	//TODO: FIX BUG
	if (!node->left && !node->right) {
		*get_parent_ptr(node, interval) = 0;
		delete node;
	}
	// Nur ein Kind
	else if (node->left && !node->right) {
		*get_parent_ptr(node, interval) = node->left;
		node->left->parent = node->parent;
		delete node;
	} else if (!node->left && node->right) {
		*get_parent_ptr(node, interval) = node->right;
		node->right->parent = node->parent;
		delete node;
	}
	// Zwei Kinder
	else {
		std::cout << "swap" << std::endl;
		pcNode*other = get_prev_node(node);
		std::cout << other->y << std::endl;
		swap_nodes(node, other, interval);
		std::cout << "after swap" << std::endl;
		// Löschen des Knoten durch Benutzen von einer
		// der beiden anderen Methoden
		remove(node, interval);
	}
}


pcNode* IntervalMap::getPCfromInterval(int interval, int index)
{
	node* intvl = getInterval(map, interval);
	int count = 0;
	return getPC(intvl->tracks, index, count);
}

pcNode* IntervalMap::getPC(pcNode* leaf, int index, int& count)
{
	if (leaf== NULL) {
		return NULL;
	}
	if (leaf->left != NULL) {
		pcNode* tmp = getPC(leaf->left, index, count);
		if(tmp)
		{
			return tmp;
		}
	}
	if(count == index)
	{
		return leaf;
	}
	++count;
	if (leaf->right != NULL) {
		pcNode* tmp = getPC(leaf->right, index, count);
		if(tmp)
		{
			return tmp;
		}
	}
	return NULL;
}

node* IntervalMap::at(int interval)
{
	return getInterval(map, interval);
}

pcNode* IntervalMap::insertNewTrack(PointCell vehicle)
{
	double x = vehicle.getX();
	int interval = floor(x);
	int realinterval = interval + CARINTERVAL;

	if(realinterval > 99 || realinterval < 0)
	{
		return NULL;
	}

	vehicle.subInvtl = (vehicle.getX() - interval);
	vehicle.setX(((double) interval)+0.5);

	return insertPCintoInterval(realinterval, vehicle);
}

void IntervalMap::removeVehicle(pcNode* node)
{
	double x = node->vehicle.getX();
	int interval = floor(x);
	interval += CARINTERVAL;

	if(interval > 99 || interval < 0)
	{
		return;
	}

	remove(node,interval);
}
