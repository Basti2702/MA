/*
 * DataVisualizer.cpp
 *
 *  Created on: 21.06.2016
 *      Author: basti
 */

#include "DataVisualizer.h"

DataVisualizer::DataVisualizer() {
	// TODO Auto-generated constructor stub
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
	// TODO Auto-generated destructor stub
}
/*
 * Creates an SVG File (http://www.w3schools.com/svg/svg_inhtml.asp) to visualize the segments as pointcloud
 */
void DataVisualizer::visualizeSegmentsAsPointCloud(std::vector<cartesian_segment> segments, std::string number)
{
	  std::ofstream myfile;
	  std::ostringstream filename;
	  filename << "./Visualization/PointCloud" << number << ".html";
	  myfile.open (filename.str().c_str());
	  myfile << "<!DOCTYPE html>" << std::endl;
	  myfile << "<html>" << std::endl;
	  myfile << "<body>" << std::endl;
	  myfile << "<svg width=\""<< CANVASSIZE <<"\" height=\"" << CANVASSIZE << "\">" << std::endl;
	  for(uint i=0; i<segments.size(); i++)
	  {
		cartesian_segment seg = segments.at(i);
		for(int j=0; j<seg.numberOfMeasures; j++)
		{
			laserdata_cartesian data = seg.measures.at(j);
			//compute coordinates on Canvas, default 0 is on top but should be on the bottom and in the middle
			int xOnCanvas = CANVASSIZE;
			int yOnCanvas = CANVASSIZE/2;

			xOnCanvas -= (data.x * CANVASFACTOR);
			yOnCanvas += (data.y *CANVASFACTOR);
			myfile << "    <circle cx=\""<< yOnCanvas << "\" cy=\"" << xOnCanvas << "\" r=\"1\" stroke=\"" << colors[i]<<"\" stroke-width=\"4\" fill=\"" << colors[i]<<"\"/>" << std::endl;
		//	myfile << "    <circle cx=\""<< yOnCanvas << "\" cy=\"" << xOnCanvas << "\" r=\"1\" stroke=\"blue\" stroke-width=\"4\" fill=\"blue\"/>" << std::endl;

		}
	  }
	  myfile << "</svg>" << std::endl;
	  myfile << "</body>" << std::endl;
	  myfile << "</html>" << std::endl;
	  myfile.close();
}



