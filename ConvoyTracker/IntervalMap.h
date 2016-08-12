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
#include "PointCell.h"




struct pcNode
{
	double y;
	PointCell vehicle;
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
	pcNode* insertPCintoInterval(int interval, PointCell vehicle);
	void deletePCfromInterval(int interval, PointCell vehicle);
	int inorderTracks(int interval);
	node* at(int interval);
	int inorderPC(pcNode* root, int print);
	pcNode* getPCfromInterval(int interval, int index);
	pcNode* insertNewTrack(PointCell vehicle);
	void removeVehicle(pcNode* node);
	void remove(pcNode*node, int interval);
	pcNode* insertPC(pcNode* leaf, PointCell vehicle);
	pcNode* getPC(pcNode* leaf, int index, int& count);

private:
	void allocateIntervalMap();
	void freeIntervalMap();
	void insertInterval(node* leaf, double key);
	void destroy_tree(node* leaf);
	void destroy_PCTree(pcNode* root);

	void deletePC(pcNode* leaf,PointCell vehicle, int interval);
	node* getInterval(node *leaf, double Intervall);

	void swap_near_nodes(pcNode*child, pcNode*parent, int interval);
	void swap_far_nodes(pcNode*a, pcNode*b, int interval);
	void swap_nodes(pcNode*a, pcNode*b, int interval);
	bool is_left_child(const pcNode*node);
	bool is_right_child(const pcNode*node);
	pcNode* get_max(pcNode*node);
	pcNode* get_prev_node(pcNode*now);

	pcNode** get_parent_ptr(pcNode*_node, int interval);


	void inorder(node* tree);


	int numberOfIntervals;
	int intervalLength;
	double xSubInterval;
	node *map;



};
#endif /* INTERVALMAP_H_ */
