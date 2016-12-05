/*
 * DataVisualizer.cpp
 *
 *  Created on: 21.06.2016
 *      Author: Sebastian Reinhart
 */

#include "DataVisualizer.cuh"

DataVisualizer::DataVisualizer() {
	//define 20 differnet colors
	colors[0] = "#ff0000";
	colors[1] = "#ff8000";
	colors[2] = "#ffff00";
	colors[3] = "#00ff00";
	colors[4] = "#00ffff";
	colors[5] = "#0040ff";
	colors[6] = "#bf00ff";
	colors[7] = "#ff0080";
	colors[8] = "#808080";
	colors[9] = "#000000";
	colors[10] = "#ffc1ce";
	colors[11] = "#8b0000";
	colors[12] = "#cccccc";
	colors[13] = "#bb4444";
	colors[14] = "#6e4474";
	colors[15] = "#00819c";
	colors[16] = "#bab206";
	colors[17] = "#af55ba";
	colors[18] = "#ff6666";
	colors[19] = "#ff15ab";
}

DataVisualizer::~DataVisualizer() {

}
/*
 * Creates an SVG File (http://www.w3schools.com/svg/svg_inhtml.asp) to visualize the segments as pointcloud
 */
void DataVisualizer::visualizeSegmentsAsPointCloud(cartesian_segment* segments, std::string number, int segment_count)
{
	  std::ofstream myfile;
	  std::ostringstream filename;
	  filename << VISUALIZATIONPATH << "/PointCloud" << number << ".html";
	  myfile.open (filename.str().c_str());
	  myfile << "<!DOCTYPE html>" << std::endl;
	  myfile << "<html>" << std::endl;
	  myfile << "<body>" << std::endl;
	  myfile << "<svg width=\""<< CANVASSIZE <<"\" height=\"" << CANVASSIZE << "\">" << std::endl;
	  for(uint i=0; i<segment_count; i++)
	  {
		cartesian_segment seg = segments[i];
		for(int j=0; j<seg.numberOfMeasures; j++)
		{
			laserdata_cartesian data = seg.measures[j];
			//compute coordinates on Canvas, default 0 is on top but should be on the bottom and in the middle for better understanding
			int xOnCanvas = CANVASSIZE;
			int yOnCanvas = CANVASSIZE/2;

			xOnCanvas -= (data.x * CANVASFACTOR);
			yOnCanvas += (data.y *CANVASFACTOR);

			std::string color;
			if(i > 19)
			{
				color = colors[19];
			}
			else
			{
				color = colors[i];
			}

			myfile << "    <circle cx=\""<< yOnCanvas << "\" cy=\"" << xOnCanvas << "\" r=\"1\" stroke=\"" << color <<"\" stroke-width=\"4\" fill=\"" << color <<"\"/>" << std::endl;
		}
	  }
	  myfile << "</svg>" << std::endl;
	  myfile << "</body>" << std::endl;
	  myfile << "</html>" << std::endl;
	  myfile.close();
}
/*
 * draws the extracted vehicles as a set of lines defined with the 3 relevant points
 */
void DataVisualizer::visualizeVehiclesAsRectangle(std::vector<std::vector<laserdata_cartesian> > segments, std::string number)
{
	std::ofstream myfile;
	std::ostringstream filename;
    filename << VISUALIZATIONPATH << "/Vehicles" << number << ".html";
	myfile.open(filename.str().c_str());
	myfile << "<!DOCTYPE html>" << std::endl;
	myfile << "<html>" << std::endl;
	myfile << "<body>" << std::endl;
	myfile << "<svg width=\"" << CANVASSIZE << "\" height=\"" << CANVASSIZE
			<< "\">" << std::endl;

	//compute coordinates on Canvas, default 0 is on top but should be on the bottom and in the middle
	int xOnCanvas = CANVASSIZE;
	int yOnCanvas = CANVASSIZE / 2;

	int ax, ay, bx, by, cx, cy;

	for (uint i = 0; i < segments.size(); i++) {
		ax = xOnCanvas - (segments.at(i).at(0).x * CANVASFACTOR);
		ay = yOnCanvas + (segments.at(i).at(0).y * CANVASFACTOR);
		bx = xOnCanvas - (segments.at(i).at(1).x * CANVASFACTOR);
		by = yOnCanvas + (segments.at(i).at(1).y * CANVASFACTOR);
		cx = xOnCanvas - (segments.at(i).at(2).x * CANVASFACTOR);
		cy = yOnCanvas + (segments.at(i).at(2).y * CANVASFACTOR);

		xOnCanvas -= (0 * CANVASFACTOR);
		yOnCanvas += (0 * CANVASFACTOR);
		myfile << "<polyline points=\"" << ay << "," << ax << " " << by << ","
				<< bx << " " << cy << "," << cx
				<< "\" style=\"fill:none;stroke:" << colors[i]
				<< ";stroke-width:3\" />" << std::endl;
		//	myfile << "    <circle cx=\""<< yOnCanvas << "\" cy=\"" << xOnCanvas << "\" r=\"1\" stroke=\"blue\" stroke-width=\"4\" fill=\"blue\"/>" << std::endl;
	}
	myfile << "</svg>" << std::endl;
	myfile << "</body>" << std::endl;
	myfile << "</html>" << std::endl;
	myfile.close();
}
/*
 * draw all convoys as a polyline defined with the position vector of each convoy
 */
void DataVisualizer::visualizeConvoys(std::vector<EMLPos> EML, Convoy* convoys, int startIndexConvoys, int endIndexConvoys)
{
	int xOnCanvas = CANVASSIZE;
	int yOnCanvas = CANVASSIZE/2;
	std::ofstream myfile;
	std::ostringstream filename;
	filename << VISUALIZATIONPATH << "/Convoys.html";
	myfile.open(filename.str().c_str());
	myfile << "<!DOCTYPE html>" << std::endl;
	myfile << "<html>" << std::endl;
	myfile << "<body>" << std::endl;
	myfile << "<svg width=\"" << CANVASSIZE << "\" height=\"" << CANVASSIZE
			<< "\">" << std::endl;

	//visualize Convoys
	EMLPos lastPos = EML.at(EML.size() -1);
	for (uint i = startIndexConvoys; i != endIndexConvoys; i = (i + 1) % NUM_CONV)
	{
		Convoy curConv = convoys[i];
		myfile << "<polyline points=\"";
		for(uint j = curConv.startIndexTracks; j != curConv.endIndexTracks; j = (j+1)%MAX_LENGTH_HIST_CONV)
		{
			EMLPos curPos =  curConv.tracks[j];
			double x = curPos.x;
			double y = curPos.y;
			xOnCanvas = CANVASSIZE;
			yOnCanvas = CANVASSIZE/2;

			xOnCanvas -= ((x + lastPos.x));
			yOnCanvas += ((y + lastPos.y));

			myfile << yOnCanvas << "," << xOnCanvas << " ";

		}
		myfile << "\" style=\"fill:none;stroke:" << colors[i] << ";stroke-width:4\" />" << std::endl;
	}

	//visualize Vehicle Motion
	myfile << "<polyline points=\"";
	for(uint i = 0; i<EML.size(); i++)
	{
		EMLPos curPos = EML.at(i);

		xOnCanvas = CANVASSIZE;
		yOnCanvas = CANVASSIZE/2;

		xOnCanvas -= ((curPos.x));
		yOnCanvas += ((curPos.y));

		myfile << yOnCanvas << "," << xOnCanvas << " ";
	}
	myfile << "\" style=\"fill:none;stroke:green;stroke-width:4\" />" << std::endl;
	myfile << "</svg>" << std::endl;
	myfile << "</body>" << std::endl;
	myfile << "</html>" << std::endl;
	myfile.close();
}

/*
 * visualize all stored histories as lines defined with the position vectors of each history
 */
void DataVisualizer::visualizeHistory(std::vector<EMLPos> EML, History* history, int startIndex, int endIndex)
{
	int xOnCanvas = CANVASSIZE;
	int yOnCanvas = CANVASSIZE/2;
	std::ofstream myfile;
	std::ostringstream filename;
	filename << VISUALIZATIONPATH << "/Historys.html";
	myfile.open(filename.str().c_str());
	myfile << "<!DOCTYPE html>" << std::endl;
	myfile << "<html>" << std::endl;
	myfile << "<body>" << std::endl;
	myfile << "<svg width=\"" << CANVASSIZE << "\" height=\"" << CANVASSIZE
			<< "\">" << std::endl;


	//visualize Vehicle Motion
	myfile << "<polyline points=\"";
	for(uint i = 0; i<EML.size(); i++)
	{
		EMLPos curPos = EML.at(i);

		xOnCanvas = CANVASSIZE;
		yOnCanvas = CANVASSIZE/2;

		xOnCanvas -= ((curPos.x) * CANVASFACTOR);
		yOnCanvas += ((curPos.y) * CANVASFACTOR);

		myfile << yOnCanvas << "," << xOnCanvas << " ";
	}
	myfile << "\" style=\"fill:none;stroke:green;stroke-width:4\" />" << std::endl;

	EMLPos lastPos = EML.at(EML.size() -1);
	for (int i = startIndex; i != endIndex; i = (i+1)%NUM_HIST)
	{
		int index = history[i].ID;
		if(index > 20)
		{
			index = 20;
		}
		myfile << "<polyline points=\"";
		for (uint j = history[i].startIndex; j != history[i].endIndex; j = (j+1)%MAX_LENGTH_HIST_CONV)
		{
			EMLPos curPos = history[i].tracks[j];
			xOnCanvas = CANVASSIZE;
			yOnCanvas = CANVASSIZE/2;

			xOnCanvas -= ((curPos.x + lastPos.x) * CANVASFACTOR);
			yOnCanvas += ((curPos.y + lastPos.y) * CANVASFACTOR);

			myfile << yOnCanvas << "," << xOnCanvas << " ";
		}
		myfile << "\" style=\"fill:none;stroke:" << colors[index-1] << ";stroke-width:4\" />" << std::endl;
	}
	myfile << "</svg>" << std::endl;
	myfile << "</body>" << std::endl;
	myfile << "</html>" << std::endl;
	myfile.close();
}
