/*
 * PGMReader.cpp
 *
 *  Created on: 14.07.2016
 *      Author: basti
 */

#include "PGMReader.h"

PGMReader::PGMReader() {
	// TODO Auto-generated constructor stub
	numRow = 0;
	numCol = 0;
	max_grey = 0;
	vehicleX = 545;
}

PGMReader::~PGMReader() {
	// TODO Auto-generated destructor stub
}

int** PGMReader::readPGMFile(const char* filename)
{
    FILE *pgmFile;
    char version[3];
    int col, row, max_gray,carX;
    int i, j;
    int lo, hi;

    pgmFile = fopen(filename, "rb");
    if (pgmFile == NULL) {
        perror("cannot open file to read");
        exit(EXIT_FAILURE);
    }

    fgets(version, sizeof(version), pgmFile);
    if (strcmp(version, "P5")) {
        fprintf(stderr, "Wrong file type!\n");
        exit(EXIT_FAILURE);
    }

    skipComments(pgmFile);
    fscanf(pgmFile, "%d", &col);
    skipComments(pgmFile);
    fscanf(pgmFile, "%d", &row);
    skipComments(pgmFile);
    fscanf(pgmFile, "%d", &max_gray);
    skipComments(pgmFile);
  //  fscanf(pgmFile, "%d", &carX);
  //  fgetc(pgmFile);

    fprintf(stdout,"Create Matrix with %d cols and %d rows, max_grey = %d CarX = %d\n", col, row, max_gray, carX);

    int** matrix = allocate_dynamic_matrix(row, col);
    numRow = row;
    numCol = col;
    max_grey = max_gray;
 //   vehicleX = carX;
    if (max_gray > 255)
    {
        for (i = 0; i < row; ++i)
        {
            for (j = 0; j < col; ++j) {
                hi = fgetc(pgmFile);
                lo = fgetc(pgmFile);
                matrix[i][j] = (hi << 8) + lo;
            }
    	}
    }
    else
    {
        for (i = 0; i < row; ++i)
        {
            for (j = 0; j < col; ++j) {
                lo = fgetc(pgmFile);
                matrix[i][j] = lo;
            }
        }
    }
    fclose(pgmFile);
    return matrix;
}

void PGMReader::skipComments(FILE *fp)
{
    int ch;
    char line[100];

    while ((ch = fgetc(fp)) != EOF && isspace(ch))
        ;
    if (ch == '#') {
        fgets(line, sizeof(line), fp);
        skipComments(fp);
    } else
        fseek(fp, -1, SEEK_CUR);
}

int **PGMReader::allocate_dynamic_matrix(int row, int col)
{
    int **ret_val;
    int i;

    ret_val = (int **)malloc(sizeof(int *) * row);
    if (ret_val == NULL) {
        perror("memory allocation failure");
        exit(EXIT_FAILURE);
    }

    for (i = 0; i < row; ++i) {
        ret_val[i] = (int *)malloc(sizeof(int) * col);
        if (ret_val[i] == NULL) {
            perror("memory allocation failure");
            exit(EXIT_FAILURE);
        }
    }

    return ret_val;
}

void PGMReader::deallocate_dynamic_matrix(int **matrix, int row)
{
    int i;

    for (i = 0; i < row; ++i)
        free(matrix[i]);
    free(matrix);
}

void PGMReader::bresenham(int** image, int x1, int y1, int x2, int y2)
{
    int delta_x(x2 - x1);
    // if x1 == x2, then it does not matter what we set here
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = std::abs(delta_x) << 1;

    int delta_y(y2 - y1);
    // if y1 == y2, then it does not matter what we set here
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = std::abs(delta_y) << 1;

 //   plot(x1, y1);

    if (delta_x >= delta_y)
    {
        // error may go below zero
        int error(delta_y - (delta_x >> 1));

        while (x1 != x2)
        {
            if ((error >= 0) && (error || (ix > 0)))
            {
                error -= delta_x;
                y1 += iy;
            }
            // else do nothing

            error += delta_y;
            x1 += ix;

            if(x1 < 0 || x1 > numRow || y1 < 0 || y1 > numCol)
            {
            	//out of bounds
            	fprintf(stdout, "Laser out of bound!\n");
            	return;
            }
            else if(image[x1][y1] == max_grey)
            {
            	//obstacle hit
            	fprintf(stdout, "Obstacel hit at x = %d, y = %d\n", x1, y1);
            	return;
            }
//            plot(x1, y1);
        }
    }
    else
    {
        // error may go below zero
        int error(delta_x - (delta_y >> 1));

        while (y1 != y2)
        {
            if ((error >= 0) && (error || (iy > 0)))
            {
                error -= delta_y;
                x1 += ix;
            }
            // else do nothing

            error += delta_x;
            y1 += iy;

            if(x1 < 0 || x1 > numRow || y1 < 0 || y1 > numCol)
            {
            	//out of bounds
            	fprintf(stdout, "Laser out of bound!\n");
            	return;
            }
            else if(image[x1][y1] != max_grey)
            {
            	//obstacle hit
            	fprintf(stdout, "Obstacel hit at x = %d, y = %d\n", x1, y1);
            	return;
            }
       //     plot(x1, y1);
        }
    }
}

/**
 * simulates 581 laserrays on the image matrix using bresenham algorithm
 * 1px = 1cm
 *
 * Car position in matrix: (545, 9999) (y, x)
 */
void PGMReader::simulateLaserRays()
{
	std::string filename = "/home/basti/MA/ConvoyTracker/Laserdata/Laser1.pgm";
	int** image = readPGMFile(filename.c_str());
	if(image == NULL)
	{
		fprintf(stderr, "COULD NOT READ FILE!");
		exit(EXIT_FAILURE);
	}
	//angel from -72.5 - 72.5 in 0.25 steps
	double angle;
	int startX = 9999;
	int startY = 545;
	int vecX = 1;
	int vecY = 0;
	for(angle = -72.5; angle <= 72.5; angle+=0.25)
	{
		//compute vector for current angle
		double angleInRadians = angle*M_PI/180.0;
		double endX = vecX * cos(angleInRadians) + vecY* sin(angleInRadians);
		double endY = -vecX * sin(angleInRadians) + vecY* cos(angleInRadians);
		fprintf(stdout, "Angle: %f, Resulting X = %f Y = %f\n", angle, endX, endY);

		//vector should point to the left side for negative values
		int stepsToBorder;
		if(angle < 0)
		{
			endY *= -1;
			stepsToBorder = vehicleX;

			//compute intersection of vector with image border
			double tmpX = endX;
			double tmpY = endY;
			endY = 0;
			endX = 9999 + stepsToBorder/(tmpY/tmpX);
	/*		if(endX < 0)
			{
				endY = vehicleX + tmpX*9999;
				endX = 0;
			}*/
		}
		else if(angle > 0)
		{
			stepsToBorder = numCol - vehicleX;
			//compute intersection of vector with image border
			double tmpX = endX;
			double tmpY = endY;
			endY = numCol-1;
			endX = 9999 + stepsToBorder/(tmpY/tmpX);
		}
		else
		{
			endX = 0;
			endY = 545;
		}

		bresenham(image,startX, startY, endX, endY);
	}

	//free memory
	deallocate_dynamic_matrix(image, numRow);
}
