/*
 * PGMReader.cpp
 *
 *  Created on: 14.07.2016
 *      Author: Sebastian Reinhart
 */

#include "PGMReader.cuh"

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

    pgmFile = fopen(filename, "rb");
    if (pgmFile == NULL) {
        perror("cannot open file to read");
        exit(EXIT_FAILURE);
    }

    if(!fgets(version, sizeof(version), pgmFile))
    {
        perror("cannot open file to read");
        exit(EXIT_FAILURE);
    }
    if (strcmp(version, "P2")) {
        fprintf(stderr, "Wrong file type!\n");
        exit(EXIT_FAILURE);
    }

    skipComments(pgmFile);
    if(!fscanf(pgmFile, "%d", &col))
    {
        perror("cannot open file to read");
        exit(EXIT_FAILURE);
    }
    skipComments(pgmFile);
    if(!fscanf(pgmFile, "%d", &row))
    {
        perror("cannot open file to read");
        exit(EXIT_FAILURE);
    }
    skipComments(pgmFile);
    if(!fscanf(pgmFile, "%d", &max_gray))
    {
        perror("cannot open file to read");
        exit(EXIT_FAILURE);
    }
    skipComments(pgmFile);
    if(!fscanf(pgmFile, "%d", &carX))
    {
        perror("cannot open file to read");
        exit(EXIT_FAILURE);
    }
    if(!fgetc(pgmFile))
    {
        perror("cannot open file to read");
        exit(EXIT_FAILURE);
    }

    fprintf(stdout,"Create Matrix with %d cols and %d rows, max_grey = %d CarX = %d\n", col, row, max_gray, carX);

    int** matrix = allocate_dynamic_matrix(row, col);
    numRow = row;
    numCol = col;
    max_grey = max_gray;
    int temp;
    vehicleX = carX;
    {
        for (i = 0; i < row; ++i)
        {
            for (j = 0; j < col; ++j) {
            	if(!fscanf(pgmFile, "%d", &temp))
            	{
                    perror("cannot open file to read");
                    exit(EXIT_FAILURE);
            	}
                matrix[i][j] = temp;
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
        if(!fgets(line, sizeof(line), fp))
        {
            perror("cannot open file to read");
            exit(EXIT_FAILURE);
        }
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

double PGMReader::bresenham(int** image, int x1, int y1, int x2, int y2)
{
    int delta_x(x2 - x1);
    // if x1 == x2, then it does not matter what we set here
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = std::abs(delta_x) << 1;

    int delta_y(y2 - y1);
    // if y1 == y2, then it does not matter what we set here
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = std::abs(delta_y) << 1;

    int startX = x1;
    int startY = y1;

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
            	return 501.0;
            }
            else if(image[x1][y1] != max_grey)
            {
            	//obstacle hit
            	double distance = sqrt((startX-x1)*(startX-x1) + (startY-y1)*(startY-y1));
            	fprintf(stdout, "Obstacle hit at x = %d, y = %d distance: %f\n", x1, y1, distance);
            	return distance;
            }
//            plot(x1, y1);
        }
        //reached target without hitting an object -> out of bound
        return 501.0;
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
            	return 501.0;
            }
            else if(image[x1][y1] != max_grey)
            {
            	//obstacle hit
            	double distance = sqrt((startX-x1)*(startX-x1) + (startY-y1)*(startY-y1));
            	fprintf(stdout, "Obstacle hit at x = %d, y = %d distance: %f\n", x1, y1, distance);
            	return distance;
            }
       //     plot(x1, y1);
        }
        //reached target without hitting an object -> out of bound
        return 501.0;
    }
}

/**
 * simulates 581 laserrays on the image matrix using bresenham algorithm
 * 1px = 1cm
 *
 * Car position in matrix: (545, 9999) (y, x)
 */
void PGMReader::simulateLaserRays(std::string number)
{
	int counter = atoi(number.c_str());
	std::ofstream laserMeasureFile;
	std::ostringstream measurePath;
	measurePath << MEASUREPATH << number << ".txt";
	laserMeasureFile.open (measurePath.str().c_str());

	measurePath.clear();
	measurePath.str("");
#if SZENARIO == 1 || SZENARIO == 5
	measurePath << MEASUREPATH << "0000.pgm";
#elif SZENARIO == 4
	if(counter < 30)
	{
		measurePath << MEASUREPATH << "0000.pgm";
	}
	else if(counter < 42)
	{
		measurePath << MEASUREPATH << number << ".pgm";
	}
	else
	{
		measurePath << MEASUREPATH << "0013.pgm";
	}
#else
	measurePath << MEASUREPATH << number << ".pgm";
#endif
	int** image = readPGMFile(measurePath.str().c_str());
	if(image == NULL)
	{
		fprintf(stderr, "COULD NOT READ FILE!");
		exit(EXIT_FAILURE);
	}
	//angel from -72.5 - 72.5 in 0.25 steps
	double angle;
	int startX = numRow-1;
	int startY = vehicleX;
	int vecX = 1;
	int vecY = 0;

	//initialize random
	srand(time(NULL));

	for(angle = -72.5; angle <= 72.5; angle+=0.25)
	{
		//we don´t care about the first 4 fields in simulation so fill it with 0
		laserMeasureFile << "0 0 0 0 ";

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
			endX = startX + stepsToBorder/(tmpY/tmpX);
		}
		else if(angle > 0)
		{
			stepsToBorder = numCol - vehicleX;
			//compute intersection of vector with image border
			double tmpX = endX;
			double tmpY = endY;
			endY = numCol-1;
			endX = startX + stepsToBorder/(tmpY/tmpX);
		}
		else
		{
			endX = 0;
			endY = vehicleX;
		}

		//compute straight line from our vehicle to the image border with given angle/vector to get the distance value for current laserray
		int valid = 1;
		double distance = bresenham(image,startX, startY, endX, endY);
		if(distance == 501.0 || distance == 0)
		{
			valid = 0;
		}
		else
		{
			//with propability of @define NOISE_RATIO percent the given distance is not valid to simulate noise
			int random = rand() % 100;
			if(random <= NOISE_RATIO)
			{
				valid = 0;
			}

		}
		//simulate uncertainty in measure up to +-10%
		double uncertainty = rand() % 10 +1;
		uncertainty /= 1000.0;
		fprintf(stdout, "Unsicherheit: %f", uncertainty);
		int sign = rand() % 2;
		if(sign)
		{
			distance += distance*uncertainty;
		}
		else
		{
			distance -= distance*uncertainty;
		}
		//change unit from cm to m
		distance /= 100.0;
		laserMeasureFile << valid << " " << distance << std::endl;

	}

	//free memory
	deallocate_dynamic_matrix(image, numRow);
	laserMeasureFile.close();
}
