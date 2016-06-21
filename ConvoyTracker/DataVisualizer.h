/*
 * DataVisualizer.h
 *
 *  Created on: 21.06.2016
 *      Author: basti
 */

#ifndef DATAVISUALIZER_H_
#define DATAVISUALIZER_H_

#include "data.h"
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <sstream>

#define CANVASSIZE 1000
#define CANVASFACTOR 10

class DataVisualizer {
public:
	DataVisualizer();
	virtual ~DataVisualizer();

	void visualizeSegmentsAsPointCloud(std::vector<cartesian_segment> segments, std::string number);

private:
	std::string colors[20];
};

#endif /* DATAVISUALIZER_H_ */
