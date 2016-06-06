/*
 * data.h
 *
 *  Created on: 06.06.2016
 *      Author: Sebastian Reinhart
 */

#ifndef DATA_H_
#define DATA_H_

typedef struct laserdata_raw{
	double angle;
	double distance;
	int valid;
}laserdata_raw_array[581];



#endif /* DATA_H_ */
