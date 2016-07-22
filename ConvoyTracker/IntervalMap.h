/*
 * IntervallMap.h
 *
 *  Created on: 27.06.2016
 *      Author: Sebastian Reinhart
 */

#ifndef INTERVALMAP_H_
#define INTERVALMAP_H_

#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <list>
#include "data.h"

#define NUMBER_OF_INTERVALS 100
#define INTERVALL_LENGTH 1 //unit m


struct pcNode
{
	double y;
	PC vehicle;
	pcNode *left;
	pcNode *right;
	pcNode *parent;
};

struct node
{
  double key_value;
  pcNode* tracks;
  node *left;
  node *right;
};



class IntervalMap {
public:
	IntervalMap();
	IntervalMap(int numberOfIntervals, int length);
	virtual ~IntervalMap();

	void shiftStructure(double xMotion);
	void rotateStructure(double angle, double yMotion);
	void insertPCintoInterval(int interval, PC vehicle);
	void deletePCfromInterval(int interval, PC vehicle);
	void inorderTracks(int interval);

private:
	void allocateIntervalMap();
	void freeIntervalMap();
	void insertInterval(node* leaf, double key);
	void destroy_tree(node* leaf);
	void destroy_PCTree(pcNode* root);
	void insertPC(pcNode* leaf, pcNode* parent, PC vehicle);
	void deletePC(pcNode* leaf,PC vehicle, int interval);
	void swap_near_nodes(pcNode*child, pcNode*parent, int interval);
	void swap_far_nodes(pcNode*a, pcNode*b, int interval);
	void swap_nodes(pcNode*a, pcNode*b, int interval);
	bool is_left_child(const pcNode*node);
	bool is_right_child(const pcNode*node);
	pcNode* get_max(pcNode*node);
	pcNode* get_prev_node(pcNode*now);
	void remove(pcNode*node, int interval);
	pcNode** get_parent_ptr(pcNode*_node, int interval);
	node* getInterval(node *leaf, double Intervall);

	void inorder(node* tree);
	void inorder(pcNode* root);

	int numberOfIntervals;
	int intervalLength;
	double xSubInterval;
	node *map;



};
#endif /* INTERVALMAP_H_ */
