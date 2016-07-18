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
	void deletePC(pcNode* leaf,PC vehicle);
	node* getInterval(node *leaf, double Intervall);

	void inorder(node* tree);
	void inorder(pcNode* root);

	int numberOfIntervals;
	int intervalLength;
	double xSubInterval;
	node *map;


	//helper methods for removing elements from pcNode
	void swap_near_nodes(pcNode*child, pcNode*parent){
	    // Als erstes passen wir den unbeteiligten Großelternknoten an.
	    *get_parent_ptr(parent) = child;

	    // Anschließend werden die Kind- und Elternzeiger ausgetauscht.
	    std::swap(parent->left, child->left);
	    std::swap(parent->right, child->right);
	    std::swap(parent->parent, child->parent);

	    // Da eines der Kinder getauscht wird benötigt es eine
	    // sonder Behandlung.
	    if(child->left == child)
	        child->left = parent;
	    else
	        child->right = parent;

	    // Nun sind alle Kindzeiger richtig und die Elternzeiger können
	    // dem angepasst werden.
	    if(child->left)
	        child->left->parent = child;
	    if(child->right)
	        child->right->parent = child;
	    if(parent->left)
	        parent->left->parent = parent;
	    if(parent->right)
	        parent->right->parent = parent;

	    // Na wer ist sich noch sicher ob wir nicht
	    // bereits Zeigersalat haben? Besser testen!
	}

	void swap_far_nodes(pcNode*a, pcNode*b){
	    // Zuerst updaten wir die Zeiger der Eltern
	    *get_parent_ptr(a) = b;
	    *get_parent_ptr(b) = a;

	    // Danach der Kinder
	    if(a->left)
	        a->left->parent = b;
	    if(a->right)
	        a->right->parent = b;
	    if(b->left)
	        b->left->parent = a;
	    if(b->right)
	        b->right->parent = a;

	    // Und als letztes die der beiden Knoten
	    std::swap(a->left, b->left);
	    std::swap(a->right, b->right);
	    std::swap(a->parent, b->parent);
	}

	void swap_nodes(pcNode*a, pcNode*b){
	    if(a->parent == b)
	        swap_near_nodes(a, b);
	    else if(b->parent == a)
	        swap_near_nodes(b, a);
	    else
	        swap_far_nodes(a, b);
	}


	static bool is_left_child(const pcNode*node){
	    if(node->parent == 0)
	        return false; // Die Wurzel ist kein Kind
	    else
	        return node->parent->left == node;
	}

	static bool is_right_child(const pcNode*node){
	    if(node->parent == 0)
	        return false; // Die Wurzel ist kein Kind
	    else
	        return node->parent->right == node;
	}

	pcNode**get_parent_ptr(pcNode*node){
	    if(node->parent == 0)
	        return NULL;
	    else if(is_left_child(node))
	        return &node->parent->left;
	    else
	        return &node->parent->right;
	}

	static pcNode*get_max(pcNode*node){
	    pcNode*now = node;
	    while(now->right)
	        now = now->right;
	    return now;
	}

	static pcNode*get_prev_node(pcNode*now){
	    if(now->left)
	        return get_max(now->left);
	    else{
	        while(now){
	            if(is_right_child(now))
	                return now->parent;
	            else
	                now = now->parent;
	        }
	        return 0; // Wir sind am Anfang angekommen
	    }
	}

	void remove(pcNode*node){
	    // Ein Blatt
	    if(!node->left && !node->right){
	        *get_parent_ptr(node) = 0;
	        delete node;
	    }
	    // Nur ein Kind
	    else if(node->left && !node->right){
	        *get_parent_ptr(node) = node->left;
	        node->left->parent = node->parent;
	        delete node;
	    }else if(!node->left && node->right){
	        *get_parent_ptr(node) = node->right;
	        node->right->parent = node->parent;
	        delete node;
	    }
	    // Zwei Kinder
	    else{
	        pcNode*other = get_prev_node(node);
	        swap_nodes(node, other);
	        // Löschen des Knoten durch Benutzen von einer
	        // der beiden anderen Methoden
	        remove(node);
	    }
	}
};
#endif /* INTERVALMAP_H_ */
