/*
 * PGMReader.h
 *
 *  Created on: 14.07.2016
 *      Author: basti
 */

#ifndef PGMREADER_H_
#define PGMREADER_H_

#include <stdio.h>
#include <stdlib.h>
#include <error.h>
#include <string.h>
#include <ctype.h>
#include <cstdlib>
#include <math.h>
#include <string>

class PGMReader {
public:
	PGMReader();
	virtual ~PGMReader();


	void deallocate_dynamic_matrix(int **matrix, int row);
	void simulateLaserRays();


private:
	void skipComments(FILE *fp);
	int **allocate_dynamic_matrix(int row, int col);
	void bresenham(int** image, int x1, int y1, int x2, int y2);
	int** readPGMFile(const char *filename);

	int numRow;
	int numCol;
	int max_grey;
	int vehicleX;

};

#endif /* PGMREADER_H_ */