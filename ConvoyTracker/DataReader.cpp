/*
 * DataReader.cpp
 *
 *  Created on: 06.06.2016
 *      Author: Sebastian Reinhart
 */

#include "DataReader.h"
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

DataReader::DataReader() {
	// TODO Auto-generated constructor stub

}

DataReader::~DataReader() {
	// TODO Auto-generated destructor stub
}

/*
 * Reads the data out of the specified file and writes it to given array
 * @param
 * 	data: array for raw laserdata
 * @return
 * 	1 if an error occurs
 * 	0 if everything is ok
 */
int DataReader::getLaserData(laserdata_raw_array data)
{
    std::ifstream input("/home/basti/MA/ConvoyTracker/Laserdata/laser1Messung.txt");
    std::string line;
    int counter = 0;
	std::string segment;

    //Skip first 581 lines, just read out the second level of datas
    while(counter < 581 && std::getline( input, line )) {
    	++counter;
    	std::cout<< counter <<'\n';
    }

    //now read the data we are interested in
    counter = 0;
    while( std::getline( input, line ) && counter < 581 ) {
    	++counter;
    	//std::cout<<line<<'\n';
    	std::stringstream ss;

    	ss << line;

    	//extract relevant data from line
    	while(std::getline(ss, segment, ' '))
    	{
    		if(segment.size() > 0)
    		{
    			if(segment.at(0) < 48 || segment.at(0) > 57)
    			{
    				continue;
    			}
    			else
    			{
    				std::cout<<segment << ' ';
    			}
    		}

    	}
    	std::cout<<'\n';
    }

    return 0;
}
